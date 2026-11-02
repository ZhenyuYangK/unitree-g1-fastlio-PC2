/* Exercise the actual browser recovery functions without connecting to a robot. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(`${__dirname}/../frontend/app.js`, 'utf8');
const recovery = source.slice(source.indexOf('function acceptState('), source.indexOf('function svgPoint('));
const renew = source.slice(source.indexOf('async function renew('), source.indexOf('function resumeControl('));
const resume = source.slice(source.indexOf('function resumeControl('), source.indexOf('document.addEventListener("visibilitychange"'));
const apiSource = source.slice(source.indexOf('async function api('), source.indexOf('function view('));

function context() {
  const c = vm.createContext({assert, console, queueMicrotask, AbortController});
  vm.runInContext(`
    let now = 3000, connected = true, lastReceived = 500, snapshotReceivedAt = 0;
    let snapshot = {}, socketStartedAt = 500, lastSocketReceived = 500, lastSocketRecovery = -10000;
    let statePoll = null, statePollController = null, nextStatePoll = 0, stateWasStale = false;
    let preferHTTPState = false, httpStateMode = false;
    let instanceId = 'instance', sequence = 1, retryDelay = 1000, retryTimer = null;
    let leaseId = 'original', haveLease = true, leaseVersion = 1, leaseValidUntil = 10000, renewal = null;
    let catalog = {revision:'r'}, mapMetadata = null, catalogRetryAt = 0, mapRetryAt = 0;
    let document = {hidden:false}, login = {open:false};
    let performance = {now:() => now}, events = [], calls = [], toasts = [];
    let $ = () => login, render = () => {}, loadCatalog = () => {}, loadMap = () => {};
    let reportConnection = (event, data) => events.push([event, data]);
    let toast = message => toasts.push(message);
    let clearTimeout = () => {}, setTimeout = fn => {queueMicrotask(fn); return 1;};
    function clearLease() {leaseVersion++; leaseId=''; haveLease=false;}
    function message(seq=2) {return {type:'state', schema_version:1, instance_id:'instance', sequence:seq,
      data:{simulated:true, configuration:{available:false}}};}
    let api = async path => {calls.push(path); return path==='/state' ? message() : {remaining_sec:300};};
    class WebSocket {
      constructor() {this.events={}; this.closed=false;}
      addEventListener(name, fn) {this.events[name]=fn;}
      close() {this.closed=true;}
    }
    let location = {protocol:'http:',host:'example.test'};
    let socket = new WebSocket();
  ` + recovery + renew + resume, c);
  return c;
}

function apiContext() {
  const c = vm.createContext({assert, AbortController, DOMException});
  vm.runInContext(`
    let leaseId='original', leaseVersion=1, timers=new Map(), nextTimer=0;
    let calls=[], loginShown=0, leasesCleared=0, bodyStarted=false;
    let setTimeout=(fn,ms) => {const id=++nextTimer; timers.set(id,{fn,ms}); return id;};
    let clearTimeout=id => timers.delete(id);
    let showLogin=() => loginShown++, clearLease=() => leasesCleared++;
    let fetch=async (url,options) => {
      calls.push({url,options});
      return {ok:true,json:async () => ({success:true})};
    };
  ` + apiSource, c);
  return c;
}

(async () => {
  let c = context();
  await vm.runInContext(`maintainConnection(); statePoll;`, c);
  vm.runInContext(`assert.equal(lastReceived,3000); assert.equal(connected,true);
    assert.equal(stateWasStale,false); assert.equal(leaseId,'original');
    assert.deepEqual(events.map(x=>x[0]),['state_stale','state_recovered']);`, c);

  c = context();
  vm.runInContext(`now=6500; nextStatePoll=20000; let oldSocket=socket;
    maintainConnection(); assert.notEqual(socket,oldSocket); assert.equal(oldSocket.closed,true);
    assert.equal(leaseId,'original');`, c);
  vm.runInContext(`let latest=socket; socket.events.message({data:JSON.stringify(message(3))});
    assert.equal(connected,true); assert.equal(sequence,3); assert.equal(socket,latest);`, c);

  c = context();
  await vm.runInContext(`let failures=0; api=async () => {calls.push('renew');
    if(!failures++) throw new TypeError('network'); return {remaining_sec:300};};
    Promise.all([renew(),renew()]);`, c);
  vm.runInContext(`assert.equal(calls.length,2); assert.equal(haveLease,true);
    assert.equal(leaseId,'original'); assert.equal(toasts.length,0);`, c);

  c = context();
  await vm.runInContext(`api=async () => {calls.push('renew'); clearLease();
    throw Object.assign(new Error('expired'),{status:409});}; renew();`, c);
  vm.runInContext(`assert.equal(calls.length,1); assert.equal(leaseId,''); assert.equal(haveLease,false);`, c);

  c = context();
  await vm.runInContext(`api=async () => {calls.push('renew'); throw Object.assign(new Error('busy'),{status:503});}; renew();`, c);
  vm.runInContext(`assert.equal(calls.length,2); assert.equal(haveLease,false);
    assert.equal(leaseId,'original'); assert.equal(events[0][0],'renew_failed'); assert.equal(toasts.length,1);`, c);

  c = context();
  await vm.runInContext(`let finish; api=() => new Promise(resolve=>finish=resolve);
    let waiting=renew(); leaseId='new'; leaseVersion++; haveLease=true;
    finish({remaining_sec:300}); waiting;`, c);
  vm.runInContext(`assert.equal(leaseId,'new'); assert.equal(haveLease,true); assert.equal(leaseValidUntil,10000);`, c);

  c = context();
  vm.runInContext(`document.hidden=true; maintainConnection(); assert.equal(calls.length,0);`, c);
  c = context();
  await vm.runInContext(`preferHTTPState=true; let old=socket; connect(); statePoll;`, c);
  vm.runInContext(`assert.equal(httpStateMode,true); assert.equal(socket,null);
    assert.equal(connected,true); assert.equal(calls[0],'/state');
    assert.equal(events.some(x=>x[0]==='http_state'),true);
    now=lastReceived+1000; maintainConnection(); maintainConnection(); assert.equal(calls.length,2);`, c);

  c = context();
  await vm.runInContext(`httpStateMode=true; api=async (path,options) => {
    calls.push(options); throw Object.assign(new Error('timeout'),{name:'TimeoutError'});
  }; maintainConnection(); statePoll;`, c);
  vm.runInContext(`assert.equal(calls[0].timeoutMs,1500);
    assert.equal(statePoll,null); assert.equal(statePollController,null);
    assert.equal(events.filter(x=>x[0]==='state_failed').length,1);
    assert.equal(leaseId,'original'); assert.equal(sequence,1);`, c);

  c = context();
  await vm.runInContext(`preferHTTPState=true; httpStateMode=true; leaseId=''; let finishes=[];
    api=(path,options) => {calls.push(options); return new Promise(resolve=>finishes.push(resolve));};
    maintainConnection(); let oldPoll=statePoll, oldSignal=calls[0].signal;
    resumeControl(); let newPoll=statePoll;
    assert.equal(oldSignal.aborted,true); assert.equal(calls.length,2);
    finishes[0](message(99)); oldPoll;`, c);
  vm.runInContext(`assert.equal(sequence,1); assert.equal(statePoll,newPoll);
    assert.equal(events.some(x=>x[0]==='state_failed'),false);`, c);
  await vm.runInContext(`finishes[1](message(3)); newPoll;`, c);
  vm.runInContext(`assert.equal(sequence,3); assert.equal(statePoll,null);`, c);

  c = context();
  await vm.runInContext(`httpStateMode=true; let finish;
    api=() => new Promise(resolve=>finish=resolve); maintainConnection();
    document.hidden=true; finish(message()); statePoll;`, c);
  vm.runInContext(`assert.equal(sequence,1);`, c);

  c = context();
  await vm.runInContext(`api=async (path,options) => {calls.push(options); return {remaining_sec:300};}; renew();`, c);
  vm.runInContext(`assert.equal(calls[0].timeoutMs,2500); assert.equal(haveLease,true);`, c);

  c = apiContext();
  await vm.runInContext(`let waiting=api('/state',{timeoutMs:1500});
    assert.equal(timers.get(1).ms,1500); waiting;`, c);
  vm.runInContext(`assert.equal(calls[0].options.cache,'no-store'); assert.equal(timers.size,0);`, c);

  c = apiContext();
  vm.runInContext(`fetch=async (url,options) => {
    calls.push({url,options}); return {ok:true,json:() => {
      bodyStarted=true;
      return new Promise((resolve,reject) => options.signal.addEventListener('abort',()=>reject(options.signal.reason)));
    }};
  }; let waiting=api('/state',{timeoutMs:1500}).catch(error=>error.name);`, c);
  await Promise.resolve();
  vm.runInContext(`assert.equal(bodyStarted,true); timers.get(1).fn();`, c);
  assert.equal(await vm.runInContext('waiting', c), 'TimeoutError');
  vm.runInContext(`assert.equal(calls[0].options.signal.aborted,true); assert.equal(timers.size,0);`, c);

  c = apiContext();
  await vm.runInContext(`let controller=new AbortController(); controller.abort();
    api('/state',{signal:controller.signal}).catch(error=>assert.equal(error.name,'AbortError'));`, c);
  vm.runInContext(`assert.equal(calls.length,0); assert.equal(timers.size,0);`, c);

  c = apiContext();
  vm.runInContext(`let finish, controller=new AbortController();
    fetch=() => new Promise(resolve=>finish=resolve);
    let waiting=api('/state',{signal:controller.signal}).catch(error=>error.name);
    controller.abort(); finish({ok:false,status:401,json:async()=>({})});`, c);
  assert.equal(await vm.runInContext('waiting', c), 'AbortError');
  vm.runInContext(`assert.equal(loginShown,0); assert.equal(leasesCleared,0); assert.equal(timers.size,0);`, c);

  c = apiContext();
  vm.runInContext(`fetch=async (url,options) => {
    calls.push({url,options}); return new Promise((resolve,reject)=>
      options.signal.addEventListener('abort',()=>reject(options.signal.reason)));
  }; let waiting=api('/missions/start',{method:'POST',control:true,body:{request_id:'one'}}).catch(error=>error.name);
  assert.equal(timers.get(1).ms,8000); timers.get(1).fn();`, c);
  assert.equal(await vm.runInContext('waiting', c), 'TimeoutError');
  vm.runInContext(`assert.equal(calls.length,1); assert.equal(timers.size,0); assert.equal(leasesCleared,0);`, c);
  console.log('Connection recovery passed, including state deadlines, body timeout, cancellation, late responses, serialized renewal and single command submission.');
})().catch(error => {console.error(error); process.exitCode=1;});

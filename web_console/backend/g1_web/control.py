import secrets
import time
from collections import OrderedDict


class GatewayError(Exception):
    def __init__(self, status, code, message):
        self.status, self.code, self.message = status, code, message
        super().__init__(message)


class ControlLease:
    """One controller, many viewers. Call from the gateway event loop only."""

    def __init__(self, seconds=300.0, clock=time.monotonic):
        self.seconds, self.clock = seconds, clock
        self.secret, self.owner, self.expires = "", "", 0.0

    def snapshot(self):
        remaining = max(0.0, self.expires - self.clock())
        return {"held": remaining > 0, "owner": self.owner if remaining else "",
                "remaining_sec": round(remaining, 1)}

    def acquire(self, owner):
        if self.snapshot()["held"]:
            raise GatewayError(409, "CONTROL_BUSY", "已有控制设备，请等待其释放或租约到期")
        self.secret, self.owner = secrets.token_urlsafe(32), owner
        self.expires = self.clock() + self.seconds
        return {"lease_id": self.secret, "expires_in": self.seconds}

    def require(self, secret):
        if not self.snapshot()["held"] or not secrets.compare_digest(secret.encode("utf-8"), self.secret.encode("utf-8")):
            raise GatewayError(409, "CONTROL_REQUIRED", "控制权已失效，请重新取得控制权")

    def renew(self, secret):
        self.require(secret)
        self.expires = self.clock() + self.seconds
        return self.snapshot()

    def release(self, secret):
        self.require(secret)
        self.secret, self.owner, self.expires = "", "", 0.0


class CommandCache:
    """Bounded retry deduplication; not a durable command journal."""

    def __init__(self, clock=time.monotonic, limit=256, ttl=600.0):
        self.clock, self.limit, self.ttl = clock, limit, ttl
        self.entries = OrderedDict()

    def get(self, scope, request_id, fingerprint):
        now = self.clock()
        self.entries = OrderedDict((k, v) for k, v in self.entries.items() if now - v[0] < self.ttl)
        entry = self.entries.get((scope, request_id))
        if entry is None:
            return None
        if entry[1] != fingerprint:
            raise GatewayError(409, "REQUEST_ID_CONFLICT", "同一 request_id 不能用于不同命令")
        return entry[2]

    def put(self, scope, request_id, fingerprint, result):
        self.entries[(scope, request_id)] = (self.clock(), fingerprint, result)
        while len(self.entries) > self.limit:
            self.entries.popitem(last=False)

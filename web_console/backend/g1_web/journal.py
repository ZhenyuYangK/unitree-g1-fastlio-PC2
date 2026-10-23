"""Durable admission receipts: an uncertain command is never sent twice."""
import json
from pathlib import Path
import sqlite3
import time

from .control import GatewayError


class CommandJournal:
    def __init__(self, path):
        self.path = Path(path)

    def connect(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        connection = sqlite3.connect(self.path)
        connection.execute('CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, fingerprint TEXT, result TEXT, error TEXT, created REAL)')
        return connection

    def begin(self, identifier, fingerprint):
        with self.connect() as db:
            row = db.execute('SELECT fingerprint,result,error FROM commands WHERE id=?', (identifier,)).fetchone()
            if row:
                if row[0] != fingerprint:
                    raise GatewayError(409, 'REQUEST_ID_CONFLICT', '同一 request_id 不能用于不同命令')
                if row[1]:
                    return json.loads(row[1])
                if row[2]:
                    error = json.loads(row[2])
                    raise GatewayError(*error)
                raise GatewayError(409, 'COMMAND_UNCONFIRMED', '原命令的结果尚未确认，请查看当前状态；不会重复发送')
            db.execute('INSERT INTO commands VALUES (?,?,NULL,NULL,?)', (identifier, fingerprint, time.time()))

    def finish(self, identifier, result=None, error=None):
        with self.connect() as db:
            db.execute('UPDATE commands SET result=?,error=? WHERE id=?',
                       (json.dumps(result, ensure_ascii=False) if result is not None else None,
                        json.dumps(error, ensure_ascii=False) if error else None, identifier))

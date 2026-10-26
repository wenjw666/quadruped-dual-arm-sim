"""Wall-clock base ownership. Expiry latches a hold until explicit recovery."""
import re
import secrets


class BaseLease:
    def __init__(self, ttl=3.):
        self.ttl = ttl
        self.current = None
        self.expired = False

    def expire(self, now):
        if self.current and now >= self.current['expires_at']:
            self.current = None
            self.expired = True
            return True
        return False

    def request(self, data, now):
        self.expire(now)
        op = data.get('op')
        if op == 'acquire':
            owner = data.get('owner', '')
            if not isinstance(owner, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,128}', owner):
                raise ValueError('INVALID_OWNER')
            if self.current:
                raise ValueError('RESOURCE_BUSY')
            if self.expired:
                raise ValueError('LEASE_EXPIRED_RESET_REQUIRED')
            self.current = dict(owner=owner, token=secrets.token_hex(16), mode='hold', expires_at=now+self.ttl)
        elif op in ('renew', 'mode', 'release'):
            if not self.current or data.get('token') != self.current['token']:
                raise ValueError('STALE_LEASE')
            if op == 'mode':
                if data.get('mode') not in ('hold', 'nav'):
                    raise ValueError('INVALID_LEASE_MODE')
                self.current['mode'] = data['mode']
            if op == 'release':
                self.current = None
            else:
                self.current['expires_at'] = now+self.ttl
        elif op == 'reset':
            if self.current:
                raise ValueError('RESOURCE_BUSY')
            self.expired = False
        else:
            raise ValueError('UNKNOWN_LEASE_OPERATION')
        return dict(ok=True, **(self.current or {}), expired=self.expired, ttl_wall_s=self.ttl)

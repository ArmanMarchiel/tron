"""Access control: optional API tokens.

``TRON_API_TOKENS`` = comma-separated ``name:token`` pairs.  When set, mutating endpoints (ingest,
session, faults, scenario edits, replay, record, baseline) require ``Authorization: Bearer <token>``
or ``X-API-Key``.  When unset (default for a local lab), everything is open and the actor is
"local".  Read endpoints are always open on the assumption the platform runs on a trusted cell
network; put a reverse proxy in front for anything else.
"""
from __future__ import annotations

import os

from fastapi import Header, HTTPException, Request


class Auth:
    def __init__(self, env: str | None = None):
        raw = env if env is not None else os.environ.get("TRON_API_TOKENS", "")
        self.tokens: dict[str, str] = {}
        for pair in filter(None, [x.strip() for x in raw.split(",")]):
            name, _, tok = pair.partition(":")
            if tok:
                self.tokens[tok] = name

    @property
    def enabled(self) -> bool:
        return bool(self.tokens)

    def actor(self, request: Request) -> str:
        tok = self._token(request.headers.get("authorization"), request.headers.get("x-api-key"))
        return self.tokens.get(tok, "local" if not self.enabled else "unknown")

    @staticmethod
    def _token(authorization: str | None, api_key: str | None) -> str | None:
        if authorization and authorization.lower().startswith("bearer "):
            return authorization[7:].strip()
        return api_key

    def dependency(self):
        async def _check(authorization: str | None = Header(default=None), x_api_key: str | None = Header(default=None)):
            if not self.enabled:
                return "local"
            tok = self._token(authorization, x_api_key)
            if tok not in self.tokens:
                raise HTTPException(401, "missing or invalid API token")
            return self.tokens[tok]
        return _check

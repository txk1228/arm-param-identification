"""CLI language switch for demo scripts.

Set ``PARAM_ID_LANG=zh`` (or ``zh_CN``) for Chinese console output.
Default / any other value → English.
"""

from __future__ import annotations

import os


def is_zh() -> bool:
    return os.environ.get("PARAM_ID_LANG", "en").lower().startswith("zh")


def t(en: str, zh: str) -> str:
    return zh if is_zh() else en

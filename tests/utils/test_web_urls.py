from __future__ import annotations

from rynnrcp.utils import web_urls


def test_browser_urls_show_local_and_lan_for_wildcard_bind(monkeypatch) -> None:
    monkeypatch.setattr(web_urls, "find_lan_ips", lambda: ["192.168.1.10"])

    assert web_urls.browser_urls("0.0.0.0", 28411) == [
        "http://127.0.0.1:28411/",
        "http://192.168.1.10:28411/",
    ]


def test_browser_urls_keep_explicit_bind_host() -> None:
    assert web_urls.browser_urls("192.168.1.20", 28411) == ["http://192.168.1.20:28411/"]

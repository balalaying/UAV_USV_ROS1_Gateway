from pathlib import Path


WEB = Path(__file__).resolve().parents[1] / 'web'


def test_mobile_page_uses_native_websocket_and_canvas():
    html = (WEB / 'index.html').read_text()
    javascript = (WEB / 'app.js').read_text()
    assert '<canvas' in html
    assert 'new WebSocket' in javascript
    assert 'window.location.hostname' in javascript
    assert 'localhost' not in javascript


def test_mobile_page_has_small_screen_breakpoints_and_no_framework():
    css = (WEB / 'style.css').read_text()
    html = (WEB / 'index.html').read_text()
    assert '@media(max-width:720px)' in css
    assert 'min-width:320px' in css
    assert 'vue' not in html.lower()
    assert 'react' not in html.lower()
    assert 'three.js' not in html.lower()

// captcha.html と settings.html で共有する設定の定義
// 優先順位: URLパラメータ > 設定タブ(localStorage) > 既定値
window.CAPTCHA_CONFIG = (() => {
  const KEY = 'captcha-settings';

  const FIELDS = [
    { key: 'pass',  label: 'チェックだけで通る確率', type: 'prob', def: 0.15 },
    { key: 'p',     label: '画像問題に正解したとき、そこで通過になる確率', type: 'prob', def: 0.45,
      hint: '外れると次の問題へ進みます' },
    { key: 'max',   label: '画像問題の最大回数', type: 'int', def: 5, min: 1, max: 20,
      hint: '最後の問題は、正解なら必ず通過、不正解なら失敗画面' },
    { key: 'types', label: '出す問題の種類', type: 'types', def: ['select', 'single', 'slider'] },
    { key: 'tol',   label: 'スライダーの許容誤差', type: 'prob', def: 0.03, max: 0.2, step: 0.005,
      hint: 'パズル幅に対する割合' },
    { key: 'reset', label: '結果画面から最初に戻る秒数', type: 'int', def: 8, min: 0, max: 60,
      hint: '0 で自動では戻らない' },
    { key: 'lock',  label: '結果画面でタップを受け付けない秒数', type: 'num', def: 2, min: 0, max: 10, step: 0.5 },
    { key: 'debug', label: 'タップ位置と座標を表示', type: 'bool', def: false },
  ];

  const TYPES = [
    { key: 'select',  label: '3×3 から該当する写真をすべて選ぶ' },
    { key: 'single',  label: '2×2 から1枚だけ選ぶ' },
    { key: 'slider',  label: 'スライダーでパズルを合わせる' },
    { key: 'dynamic', label: '選んだ写真が入れ替わる（該当がなくなるまで選ぶ）' },
  ];

  const defaults = () => Object.fromEntries(FIELDS.map(f => [f.key, f.def]));

  function loadSaved() {
    try { return JSON.parse(localStorage.getItem(KEY)) || {}; } catch { return {}; }
  }
  function save(values) {
    try { localStorage.setItem(KEY, JSON.stringify(values)); return true; } catch { return false; }
  }
  function clear() {
    try { localStorage.removeItem(KEY); return true; } catch { return false; }
  }

  // URL の文字列を設定値に変換
  function fromUrl(params) {
    const out = {};
    for (const f of FIELDS) {
      if (!params.has(f.key)) continue;
      const v = params.get(f.key);
      if (f.type === 'types') out[f.key] = v.split(',').filter(Boolean);
      else if (f.type === 'bool') out[f.key] = v === '1' || v === 'true';
      else if (!Number.isNaN(Number(v))) out[f.key] = Number(v);
    }
    return out;
  }

  // 実際に使う設定 (既定値 ← 保存値 ← URL)
  function resolve(search) {
    const params = new URLSearchParams(search);
    const cfg = Object.assign(defaults(), loadSaved(), fromUrl(params));
    const known = TYPES.map(t => t.key);
    cfg.types = (cfg.types || []).filter(t => known.includes(t));
    if (!cfg.types.length) cfg.types = defaults().types;
    return { cfg, urlKeys: FIELDS.map(f => f.key).filter(k => params.has(k)) };
  }

  return { KEY, FIELDS, TYPES, defaults, loadSaved, save, clear, resolve };
})();

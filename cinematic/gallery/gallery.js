(() => {
  'use strict';
  const $ = s => document.querySelector(s);
  const key = 'viscous-rov-gallery-favourites-v1';
  let shots = [], category = 'All', favouritesOnly = false, activeId = null;
  let selected = new Set();
  try { selected = new Set(JSON.parse(localStorage.getItem(key) || '[]')); } catch {}
  const visible = () => shots.filter(s => (category === 'All' || s.group === category) && (!favouritesOnly || selected.has(s.id)));
  const el = (tag, text, className) => { const n = document.createElement(tag); if (text) n.textContent = text; if (className) n.className = className; return n; };
  function announce(message) { $('#status').textContent = message; }
  function save() { try { localStorage.setItem(key, JSON.stringify([...selected].sort())); } catch {} }
  function updateSelection() {
    $('#selection-count').textContent = selected.size;
    $('#selection-summary').textContent = selected.size ? 'Favourites: ' + [...selected].sort().join(', ') : 'No favourites yet. Use the heart beside an image.';
    $('#copy-selection').disabled = !selected.size;
    $('#clear-selection').disabled = !selected.size;
  }
  function toggle(id) {
    selected.has(id) ? selected.delete(id) : selected.add(id);
    save(); render();
    if ($('#viewer').open) updateViewer();
  }
  function render() {
    const grid = $('#grid'); grid.replaceChildren();
    for (const s of visible()) {
      const card = el('article', null, 'card');
      const open = el('button', null, 'open-image'); open.setAttribute('aria-label', `Open view ${s.id}: ${s.title}`);
      const image = el('img'); image.src = s.thumbnail; image.alt = s.title; image.loading = 'lazy'; image.decoding = 'async';
      open.append(image); open.onclick = () => show(s.id); card.append(open);
      const caption = el('div', null, 'caption'); caption.append(el('span', s.id, 'number'));
      const title = el('div'); title.append(el('h2', s.title), el('p', `${s.group} · ${s.width} × ${s.height}`)); caption.append(title);
      const heart = el('button', selected.has(s.id) ? '♥' : '♡', 'heart'); heart.setAttribute('aria-pressed', selected.has(s.id));
      heart.setAttribute('aria-label', `Favourite view ${s.id}`); heart.onclick = () => toggle(s.id); caption.append(heart); card.append(caption); grid.append(card);
    }
    $('#empty').hidden = visible().length !== 0; updateSelection();
  }
  function updateViewer() {
    const s = shots.find(s => s.id === activeId); if (!s) return;
    $('#viewer-number').textContent = s.id; $('#viewer-title').textContent = s.title;
    $('#viewer-meta').textContent = `${s.group} · ${s.width} × ${s.height}`;
    $('#large-image').src = s.image; $('#large-image').alt = s.title;
    $('#download-jpg').href = s.image; $('#download-png').href = s.png;
    $('#viewer-favourite').textContent = selected.has(s.id) ? '♥ Favourited' : '♡ Favourite';
    $('#viewer-favourite').setAttribute('aria-pressed', selected.has(s.id));
  }
  function show(id) { activeId = id; updateViewer(); if (!$('#viewer').open) $('#viewer').showModal(); }
  function step(direction) {
    const list = visible(); if (!list.length) return;
    const index = list.findIndex(s => s.id === activeId);
    show(list[(index + direction + list.length) % list.length].id);
  }
  $('#close-viewer').onclick = () => $('#viewer').close();
  $('#previous').onclick = () => step(-1); $('#next').onclick = () => step(1);
  $('#viewer-favourite').onclick = () => toggle(activeId);
  $('#viewer').addEventListener('keydown', e => {
    if (e.key === 'ArrowLeft') { e.preventDefault(); step(-1); }
    if (e.key === 'ArrowRight') { e.preventDefault(); step(1); }
  });
  $('#selected-filter').onclick = () => {
    favouritesOnly = !favouritesOnly; $('#selected-filter').setAttribute('aria-pressed', favouritesOnly); render();
  };
  $('#clear-selection').onclick = () => { selected.clear(); save(); render(); announce('Favourites cleared.'); };
  $('#copy-selection').onclick = async () => {
    const text = 'My favourite ROV views: ' + [...selected].sort().join(', ');
    try {
      if (navigator.clipboard && window.isSecureContext) await navigator.clipboard.writeText(text);
      else {
        const area = el('textarea'); area.value = text; area.style.position = 'fixed'; area.style.opacity = '0'; document.body.append(area); area.select();
        const copied = document.execCommand('copy'); area.remove(); if (!copied) throw new Error('copy unavailable');
      }
      announce('Copied. Paste the numbers into our conversation.');
    } catch { announce('Select and copy the favourite numbers shown above.'); }
  };
  fetch('shots.json').then(r => { if (!r.ok) throw new Error('Shot list unavailable'); return r.json(); }).then(data => {
    shots = data; const known = new Set(shots.map(s => s.id)); selected = new Set([...selected].filter(id => known.has(id)));
    for (const name of ['All', 'At work', 'Details', 'ROV only', 'Ship context']) {
      const count = name === 'All' ? shots.length : shots.filter(s => s.group === name).length;
      const button = el('button', `${name} ${count}`); button.setAttribute('aria-pressed', name === category);
      button.onclick = () => { category = name; for (const child of $('#filters').children) child.setAttribute('aria-pressed', child === button); render(); };
      $('#filters').append(button);
    }
    render();
  }).catch(() => { $('#grid').textContent = 'The gallery could not load. Refresh the page to try again.'; });
})();

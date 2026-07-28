function renderSidebar(active) {
  const items = [
    { href: '../dashboard/index.html', label: 'Dashboard', icon: '&#9679;' },
    { href: '../maze/index.html', label: 'Live Maze', icon: '&#9635;' },
    { href: '../controls/index.html', label: 'Controls', icon: '&#9881;' },
    { href: '../history/index.html', label: 'History', icon: '&#8635;' },
    { href: '../settings/index.html', label: 'Settings', icon: '&#9881;&#65039;' }
  ];
  const user = Auth.getUser();
  return `
  <aside class="sidebar">
    <div class="brand"><span class="dot"></span><span>MAZE ROBOT LAB</span></div>
    ${items.map(i => `<a class="nav-link ${i.label === active ? 'active' : ''}" href="${i.href}">${i.label}</a>`).join('')}
    <div style="margin-top:auto; padding: 14px 12px; border-top: 1px solid var(--border-glass); font-size:12px; color:var(--text-secondary);">
      Signed in as <strong style="color:var(--text-primary)">${user?.username || ''}</strong><br>
      <span class="mono" style="text-transform:uppercase; font-size:11px;">${user?.role || ''}</span>
      <div style="margin-top:10px;"><a href="#" onclick="Auth.clear(); window.location.href='../login/index.html'; return false;">Log out</a></div>
    </div>
  </aside>`;
}
window.renderSidebar = renderSidebar;

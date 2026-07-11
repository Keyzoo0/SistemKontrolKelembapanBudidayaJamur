/* ============================================================
   Kendali Kabut — Kumbung Sukosewu
   Poll /api/data tiap 2 dtk, render dashboard + grafik ECharts,
   kirim perintah ke /api/control.
   ============================================================ */
"use strict";
const $ = id => document.getElementById(id);
const V = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

const EMBUN = V('--embun'), JERAMI = V('--jerami'), KREM = V('--krem'),
      SAGE = V('--sage'), MUTED = V('--muted'), GRID = V('--grid'),
      TANAH2 = V('--tanah-2'), WASPADA = V('--waspada'), KRITIS = V('--kritis');
const MONO = 'JetBrains Mono, monospace';

let st = null;          // state terakhir dari alat
let lastOk = 0;         // waktu fetch sukses terakhir
let dragging = false;   // slider sedang ditarik
let chart = null;

/* ---------------- util ---------------- */
function fmt(v, d = 1){ return (v == null || isNaN(v)) ? '--' : Number(v).toFixed(d); }
function fmtX(v){
  v = +v;
  if (v === 0) return 'sekarang';
  const m = Math.floor(Math.abs(v) / 60), s = Math.abs(v) % 60;
  return (v < 0 ? '-' : '+') + m + 'm' + (s ? ' ' + s + 's' : '');
}
function upFmt(s){
  if (s == null) return '--';
  const h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60);
  return (h ? h + ' jam ' : '') + m + ' mnt';
}
const WMOJI = c => c == 0 ? '☀️' : c <= 2 ? '⛅' : c == 3 ? '☁️' : c <= 48 ? '🌫️'
              : c <= 57 ? '🌦️' : c <= 67 ? '🌧️' : c <= 77 ? '🌨️' : c <= 82 ? '🌧️' : '⛈️';

/* ---------------- API ---------------- */
async function poll(){
  try{
    const r = await fetch('/api/data', { cache: 'no-store' });
    if (!r.ok) throw 0;
    st = await r.json();
    lastOk = Date.now();
    render();
  }catch(e){ renderOffline(); }
}
function ctl(p){
  fetch('/api/control?' + new URLSearchParams(p), { method: 'POST' })
    .then(() => poll()).catch(() => {});
}
function renderOffline(){
  if (Date.now() - lastOk > 6000){
    $('banOffline').classList.add('show');
    $('dotNet').className = 'dot off';
    $('chipNet').textContent = 'offline';
  }
}

/* ---------------- render DOM ---------------- */
function badge(el, cls, txt){ el.className = 'badge ' + cls; el.innerHTML = txt; }

function render(){
  if (!st) return;
  $('banOffline').classList.remove('show');
  $('dotNet').className = 'dot on';
  $('chipNet').textContent = (st.sys.ap ? 'AP ' : 'WiFi ') + st.sys.ip;

  // banner
  $('banSensor').classList.toggle('show', !st.ok);
  $('banAlarm').classList.toggle('show', st.fillAlarm);
  $('banSafety').classList.toggle('show', st.safety);
  $('dotSensor').className = 'dot ' + (st.ok ? 'on' : 'off');

  // hero
  $('heroRH').textContent = fmt(st.rh);
  $('heroMist').textContent = Math.round(st.mist.pct);
  $('aiTxt').textContent = st.ai || '—';
  const b = $('bRH');
  if (st.rh == null || !st.ok)   badge(b, 'warn', '●&nbsp;menunggu sensor');
  else if (st.rh >= 90)          badge(b, 'crit', '▲&nbsp;terlalu lembap');
  else if (st.rh >= 70)          badge(b, 'good', '✔&nbsp;ideal 70–90%');
  else                           badge(b, 'warn', '▼&nbsp;kering — di bawah 70%');

  // kabut hidup: kepekatan = PWM mist, hembusan = fan
  const fog = $('fog');
  fog.style.opacity = st.mist.pct <= 0 ? 0.05 : (0.22 + 0.78 * st.mist.pct / 100);
  fog.classList.toggle('windy', !!st.fan.on);

  // tiles
  $('vT').textContent = fmt(st.t);
  $('fT').textContent = st.ok ? 'sensor SHT31 normal' : 'sensor tidak terbaca';
  const fc = st.fc || [];
  const fcEnd = fc.length ? fc[fc.length - 1] : null;
  $('vFc').textContent = fmt(fcEnd);
  if (fcEnd != null && st.rh != null){
    const d = fcEnd - st.rh;
    $('fFc').textContent = (d >= 0 ? 'naik +' : 'turun ') + d.toFixed(1) + ' % dari sekarang';
  }
  if (st.w && st.w.ok){
    $('vW').textContent = WMOJI(st.w.code) + ' ' + fmt(st.w.t) + '°C';
    $('fW').textContent = st.w.desc + ' · RH ' + fmt(st.w.rh, 0) + '% · ' + st.w.ageS + ' dtk lalu';
  } else $('fW').textContent = 'belum ada data (cek internet)';
  $('vAir').textContent = st.waterFull ? 'Penuh' : 'Kurang';
  $('fAir').textContent = st.fillAlarm ? 'ALARM — kran ditutup paksa'
                        : (st.valve.on ? 'kran terbuka · mengisi…' : 'kran tertutup');

  // kontrol mist
  seg('segMist', st.mist.mode);
  const man = st.mist.mode === 'manual';
  const sl = $('slMist');
  sl.disabled = !man;
  if (!dragging){
    sl.value = man ? st.mist.man : st.mist.pct;
    sl.style.setProperty('--fill', sl.value + '%');
  }
  $('pctMist').textContent = Math.round(st.mist.pct) + '%';
  $('stMist').textContent = 'PWM ' + Math.round(st.mist.pct) + '% (' +
    Math.round(st.mist.pct * 1023 / 100) + '/1023)' + (st.safety ? ' · SAFETY CUT' : '');
  $('barMist').style.width = st.mist.pct + '%';

  // fan
  seg('segFan', st.fan.mode);
  $('swFan').disabled = st.fan.mode !== 'manual';
  $('swFan').checked = st.fan.mode === 'manual' ? st.fan.man : st.fan.on;
  $('stFan').textContent = st.fan.on ? 'berputar' : 'mati';

  // valve
  seg('segValve', st.valve.mode);
  $('swValve').disabled = st.valve.mode !== 'manual';
  $('swValve').checked = st.valve.mode === 'manual' ? st.valve.man : st.valve.on;
  $('stValve').textContent = st.valve.on ? 'kran terbuka' : 'kran tertutup';
  badge($('bWater'), st.waterFull ? 'good' : 'warn',
        st.waterFull ? '✔&nbsp;air penuh' : '▼&nbsp;air kurang');

  $('foot').textContent = 'bagus.local · IP ' + st.sys.ip + ' · RSSI ' + st.sys.rssi + ' dBm · heap ' +
    Math.round(st.sys.heap / 1024) + ' KB · uptime ' + upFmt(st.sys.upS) +
    ' · SistemKontrolKelembapanBudidayaJamur v2.0';

  updChart();
}
function seg(id, mode){
  $(id).querySelectorAll('button').forEach(bt =>
    bt.classList.toggle('act', bt.dataset.m === mode));
}

/* ---------------- events kontrol ---------------- */
[['segMist','mist'], ['segFan','fan'], ['segValve','valve']].forEach(([id, dev]) => {
  $(id).querySelectorAll('button').forEach(bt =>
    bt.addEventListener('click', () => ctl({ dev, mode: bt.dataset.m })));
});
$('slMist').addEventListener('input', e => {
  dragging = true;
  $('pctMist').textContent = e.target.value + '%';
  e.target.style.setProperty('--fill', e.target.value + '%');
});
$('slMist').addEventListener('change', e => { dragging = false; ctl({ dev: 'mist', val: e.target.value }); });
$('swFan').addEventListener('change', e => ctl({ dev: 'fan', val: e.target.checked ? 1 : 0 }));
$('swValve').addEventListener('change', e => ctl({ dev: 'valve', val: e.target.checked ? 1 : 0 }));
$('btnResetAlarm').addEventListener('click', () => ctl({ dev: 'alarm' }));

/* ============================================================
   GRAFIK — ECharts, dua panel (kelembapan besar + suhu kecil).
   Domain waktu -180…+300 dtk, garis SEKARANG di tengah.
   ============================================================ */
function initChart(){
  if (!window.echarts){ $('chart').hidden = true; $('chartFallback').hidden = false; return; }
  chart = echarts.init($('chart'));

  chart.setOption({
    animationDuration: 400,
    animationDurationUpdate: 550,
    animationEasingUpdate: 'cubicOut',
    axisPointer: { link: [{ xAxisIndex: 'all' }] },
    tooltip: {
      trigger: 'axis',
      backgroundColor: TANAH2,
      borderColor: 'rgba(239,233,218,.14)',
      borderWidth: 1,
      textStyle: { color: KREM, fontFamily: 'Instrument Sans, sans-serif', fontSize: 12.5 },
      axisPointer: { type: 'line', lineStyle: { color: MUTED, type: 'dashed' } },
      formatter: ps => {
        if (!ps.length) return '';
        const t = +ps[0].axisValue;
        let h = '<span style="font-family:' + MONO + ';font-size:11px;color:' + SAGE + '">' + fmtX(t) + '</span>';
        ps.forEach(p => {
          const v = p.value && p.value[1];
          if (v == null) return;
          if (p.seriesId === 'hist')      h += '<br>💧 <b>' + (+v).toFixed(1) + ' %</b>';
          else if (p.seriesId === 'fc' && t !== 0) h += '<br>💧 prediksi <b>' + (+v).toFixed(1) + ' %</b>';
          else if (p.seriesId === 'temp') h += '<br>🌡 <b>' + (+v).toFixed(1) + ' °C</b>';
        });
        return h;
      }
    },
    title: [
      { text: 'PREDIKSI AI →', right: 14, top: 0,
        textStyle: { fontSize: 9.5, color: MUTED, fontFamily: MONO, fontWeight: 600 } },
      { text: 'SUHU (°C)', left: 52, top: '69.5%',
        textStyle: { fontSize: 9.5, color: MUTED, fontFamily: MONO, fontWeight: 600 } }
    ],
    grid: [
      { left: 52, right: 56, top: 26, height: '58%' },
      { left: 52, right: 56, top: '75.5%', height: '16%' }
    ],
    xAxis: [
      { gridIndex: 0, type: 'value', min: -180, max: 300, interval: 60,
        axisLabel: { show: false }, axisLine: { show: false }, axisTick: { show: false },
        splitLine: { show: true, lineStyle: { color: GRID } } },
      { gridIndex: 1, type: 'value', min: -180, max: 300, interval: 60,
        axisLabel: { formatter: v => v === 0 ? '' : (v < 0 ? '-' : '+') + Math.abs(v) / 60 + 'm',
                     color: MUTED, fontFamily: MONO, fontSize: 10 },
        axisLine: { show: false }, axisTick: { show: false },
        splitLine: { show: true, lineStyle: { color: GRID } } }
    ],
    yAxis: [
      { gridIndex: 0, type: 'value', min: 50, max: 100, interval: 10,
        axisLabel: { formatter: '{value}%', color: MUTED, fontFamily: MONO, fontSize: 10 },
        splitLine: { lineStyle: { color: GRID } } },
      { gridIndex: 1, type: 'value', min: 20, max: 35, splitNumber: 2,
        axisLabel: { formatter: '{value}°', color: MUTED, fontFamily: MONO, fontSize: 10 },
        splitLine: { show: false } }
    ],
    series: [
      { id: 'hist', name: 'Kelembapan', type: 'line', xAxisIndex: 0, yAxisIndex: 0,
        showSymbol: false, z: 6,
        lineStyle: { width: 3, color: EMBUN, cap: 'round', join: 'round' },
        itemStyle: { color: EMBUN },
        areaStyle: { color: { type: 'linear', x: 0, y: 0, x2: 0, y2: 1, colorStops: [
          { offset: 0, color: 'rgba(31,169,143,.13)' }, { offset: 1, color: 'rgba(31,169,143,0)' } ] } },
        markArea: { silent: true, data: [
          [{ yAxis: 70, itemStyle: { color: 'rgba(99,201,123,.07)' } }, { yAxis: 90 }],
          [{ xAxis: 0,  itemStyle: { color: 'rgba(69,214,190,.045)' } }, { xAxis: 300 }]
        ] },
        markLine: { silent: true, symbol: 'none', data: [
          { yAxis: 90, lineStyle: { color: KRITIS, type: 'dashed', opacity: .75 }, label: { show: false } },
          { yAxis: 80, lineStyle: { color: MUTED, type: 'dotted', opacity: .85 }, label: { show: false } },
          { yAxis: 70, lineStyle: { color: WASPADA, type: 'dashed', opacity: .5 }, label: { show: false } },
          { xAxis: 0,  lineStyle: { color: SAGE, type: 'solid', width: 1.6, opacity: .9 },
            label: { show: true, formatter: 'SEKARANG', position: 'end', distance: 4,
                     color: KREM, fontFamily: MONO, fontSize: 9, fontWeight: 600,
                     backgroundColor: TANAH2, padding: [3, 8], borderRadius: 99,
                     borderColor: 'rgba(239,233,218,.14)', borderWidth: 1 } }
        ] },
        data: [] },
      { id: 'fc', name: 'Prediksi AI', type: 'line', xAxisIndex: 0, yAxisIndex: 0,
        showSymbol: false, z: 5,
        lineStyle: { width: 2.2, color: EMBUN, type: [7, 6] },
        itemStyle: { color: EMBUN },
        endLabel: { show: true, formatter: p => '≈' + (+p.value[1]).toFixed(1) + '%',
                    color: KREM, fontFamily: MONO, fontSize: 11, fontWeight: 600, distance: 7 },
        data: [] },
      { id: 'now', type: 'scatter', xAxisIndex: 0, yAxisIndex: 0, z: 7,
        symbolSize: 10, itemStyle: { color: EMBUN, borderColor: '#16201B', borderWidth: 2.5 },
        tooltip: { show: false }, data: [] },
      { id: 'temp', name: 'Suhu', type: 'line', xAxisIndex: 1, yAxisIndex: 1,
        showSymbol: false, z: 5,
        lineStyle: { width: 2, color: JERAMI }, itemStyle: { color: JERAMI },
        markLine: { silent: true, symbol: 'none', data: [
          { xAxis: 0, lineStyle: { color: SAGE, type: 'solid', width: 1.6, opacity: .9 }, label: { show: false } } ] },
        data: [] }
    ]
  });

  new ResizeObserver(() => chart.resize()).observe($('chart'));
}

function updChart(){
  if (!chart || !st) return;
  const hh = st.histH || [], ht = st.histT || [];
  const n = hh.length, step = st.histStep || 2;

  const hd = [], td = [];
  for (let i = 0; i < n; i++){
    const t = -(n - 1 - i) * step;
    hd.push([t, hh[i]]);
    td.push([t, ht[i]]);
  }
  const fd = [];
  if (st.rh != null && st.fc && st.fc.length){
    fd.push([0, st.rh]);
    st.fc.forEach((v, i) => fd.push([(i + 1) * (st.fcStep || 10), v]));
  }

  // skala kelembapan: turun bila data di bawah 50, tidak pernah menyempit mendadak
  const all = hh.concat(st.fc || []).filter(v => v != null);
  let lo = 50;
  if (all.length) lo = Math.max(0, Math.min(50, Math.floor((Math.min(...all) - 3) / 10) * 10));

  // skala suhu mengikuti data
  const at = ht.filter(v => v != null);
  let tLo = 20, tHi = 35;
  if (at.length){
    tLo = Math.floor(Math.min(...at) - 1);
    tHi = Math.ceil(Math.max(...at) + 1);
    if (tHi - tLo < 4){ const c = (tHi + tLo) / 2; tLo = Math.floor(c - 2); tHi = Math.ceil(c + 2); }
  }

  chart.setOption({
    yAxis: [ { min: lo, max: 100 }, { min: tLo, max: tHi } ],
    series: [
      { id: 'hist', data: hd },
      { id: 'fc',   data: fd },
      { id: 'now',  data: (st.rh != null && n) ? [[0, st.rh]] : [] },
      { id: 'temp', data: td }
    ]
  });
}

/* ---------------- init ---------------- */
initChart();
setInterval(() => {
  $('chipClock').textContent = new Date().toLocaleTimeString('id-ID');
  if (Date.now() - lastOk > 6000) renderOffline();
}, 1000);
poll();
setInterval(poll, 2000);

/* ==========================================================================
   MOON – Llamada de emergencia (función serverless de Vercel, plan gratis)

   Corre en Vercel, no en el navegador, así el Auth Token de Twilio nunca
   queda expuesto en la página.

   La página (index.html) le manda un POST (formulario) con:
     to        -> teléfono del contacto, formato E.164 (+5491112345678)
     paciente  -> nombre del paciente
     motivo    -> 'caida' | 'vitales' | 'manual'
     detalle   -> texto opcional (ej: "oxígeno en sangre")
     sms       -> '1' para mandar además un SMS (opcional)

   Variables de entorno (Vercel > tu proyecto > Settings > Environment Variables):
     TWILIO_ACCOUNT_SID  -> Account SID (empieza con AC...)           (obligatoria)
     TWILIO_AUTH_TOKEN   -> Auth Token                                 (obligatoria)
     TWILIO_FROM         -> tu número de Twilio, ej: +15017122661      (obligatoria)
     ALLOWED_ORIGINS     -> otros dominios que pueden usarla, separados por coma,
                            ej: https://proyectomoon.github.io  (opcional: la
                            página publicada en el mismo Vercel ya está permitida)
     ALLOWED_PREFIXES    -> prefijos de país permitidos, ej: +54      (opcional)
   ========================================================================== */

// Límite simple contra abusos / llamadas en loop: una llamada por número por minuto.
// (Best effort: vive en memoria mientras la función está activa.)
const MIN_GAP_MS = 60 * 1000;
const ultimas = new Map();

function escaparXml(s) {
  return String(s || '')
    .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;').replace(/'/g, '&apos;');
}

function textoMensaje(paciente, motivo, detalle) {
  const nombre = paciente || 'el paciente';
  if (motivo === 'caida') {
    return `Atención. Esta es una alerta de emergencia de la pulsera MOON. Se detectó una posible caída de ${nombre}. Por favor, comuníquese con el paciente de inmediato.`;
  }
  if (motivo === 'vitales') {
    return `Atención. Esta es una alerta de emergencia de la pulsera MOON. ${nombre} tiene un valor crítico en ${detalle || 'sus signos vitales'}. Por favor, comuníquese con el paciente de inmediato.`;
  }
  return `Atención. ${nombre} pidió ayuda desde la pulsera MOON. Por favor, comuníquese con el paciente de inmediato.`;
}

async function twilio(recurso, params) {
  const sid = process.env.TWILIO_ACCOUNT_SID;
  const auth = Buffer.from(sid + ':' + process.env.TWILIO_AUTH_TOKEN).toString('base64');
  const res = await fetch(`https://api.twilio.com/2010-04-01/Accounts/${sid}/${recurso}.json`, {
    method: 'POST',
    headers: { Authorization: 'Basic ' + auth, 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams(params)
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    const err = new Error(data.message || ('Twilio respondió ' + res.status));
    err.code = data.code;
    throw err;
  }
  return data;
}

function leerBody(req) {
  if (req.body && typeof req.body === 'object') return req.body;
  if (typeof req.body === 'string') return Object.fromEntries(new URLSearchParams(req.body));
  return {};
}

module.exports = async function handler(req, res) {
  const origin = req.headers.origin || '';
  const host = req.headers['x-forwarded-host'] || req.headers.host || '';
  const permitidos = String(process.env.ALLOWED_ORIGINS || '').split(',').map(s => s.trim()).filter(Boolean);
  let mismoSitio = false;
  try { mismoSitio = !!origin && new URL(origin).host === host; } catch (e) { /* origin inválido */ }
  const origenOk = mismoSitio || permitidos.includes(origin);

  if (origenOk) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    res.setHeader('Vary', 'Origin');
  }
  if (req.method === 'OPTIONS') return res.status(204).end();
  if (req.method !== 'POST') return res.status(405).json({ ok: false, error: 'Usá POST.' });
  if (!origenOk) return res.status(403).json({ ok: false, error: 'Origen no permitido: ' + (origin || '(vacío)') + '. Agregalo en ALLOWED_ORIGINS.' });

  if (!process.env.TWILIO_ACCOUNT_SID || !process.env.TWILIO_AUTH_TOKEN || !process.env.TWILIO_FROM) {
    return res.status(500).json({ ok: false, error: 'Faltan variables de Twilio en Vercel (TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN, TWILIO_FROM).' });
  }

  const body = leerBody(req);
  const to = String(body.to || '').replace(/[\s\-()]/g, '');
  if (!/^\+[1-9]\d{7,14}$/.test(to)) {
    return res.status(400).json({ ok: false, error: 'Teléfono inválido (usá formato +5491112345678).' });
  }
  const prefijos = String(process.env.ALLOWED_PREFIXES || '').split(',').map(s => s.trim()).filter(Boolean);
  if (prefijos.length && !prefijos.some(p => to.startsWith(p))) {
    return res.status(400).json({ ok: false, error: 'Ese país no está habilitado para llamadas.' });
  }

  const ahora = Date.now();
  if (ahora - (ultimas.get(to) || 0) < MIN_GAP_MS) {
    return res.status(429).json({ ok: false, error: 'Ya se llamó a ese número hace menos de 1 minuto.' });
  }
  ultimas.set(to, ahora);

  const paciente = String(body.paciente || '').slice(0, 60);
  const motivo = String(body.motivo || 'manual');
  const detalle = String(body.detalle || '').slice(0, 120);
  const texto = textoMensaje(paciente, motivo, detalle);

  // El mensaje se repite 3 veces por si la persona tarda en atender / escuchar.
  const voz = '<Say language="es-MX" voice="Polly.Mia">' + escaparXml(texto) + '</Say><Pause length="1"/>';
  const twiml = '<Response>' + voz + voz + voz + '</Response>';

  try {
    const call = await twilio('Calls', { To: to, From: process.env.TWILIO_FROM, Twiml: twiml });
    let smsSid = null;
    if (String(body.sms) === '1') {
      try {
        smsSid = (await twilio('Messages', { To: to, From: process.env.TWILIO_FROM, Body: texto })).sid;
      } catch (e) {
        console.error('MOON: no se pudo mandar el SMS', e.message);
      }
    }
    return res.status(200).json({ ok: true, callSid: call.sid, smsSid: smsSid });
  } catch (err) {
    ultimas.delete(to); // permitir reintento si falló
    console.error('MOON: error al crear la llamada', err.message);
    return res.status(502).json({ ok: false, error: err.message, code: err.code });
  }
};

/* ==========================================================================
   MOON – Llamada de emergencia (Twilio Function)

   Esta función corre en los servidores de Twilio (no en la página), así el
   Auth Token de Twilio nunca queda expuesto en el navegador.

   La página del paciente (index.html) le manda un POST con:
     to        -> teléfono del contacto de emergencia, formato E.164 (+5491112345678)
     paciente  -> nombre del paciente
     motivo    -> 'caida' | 'vitales' | 'manual'
     detalle   -> texto opcional (ej: "oxígeno en sangre, frecuencia cardíaca")
     sms       -> '1' para mandar además un SMS (opcional)

   Variables de entorno (Twilio Console > Functions and Assets > Services >
   tu servicio > Environment Variables):
     TWILIO_FROM       -> tu número de Twilio, ej: +15017122661   (obligatoria)
     ALLOWED_ORIGINS   -> dominios que pueden llamar a esta función, separados
                          por coma, ej: https://proyectomoon.github.io  (obligatoria)
     ALLOWED_PREFIXES  -> prefijos de país permitidos, ej: +54,+598     (opcional)
   Además dejá tildado "Add my Twilio Credentials (ACCOUNT_SID) and
   (AUTH_TOKEN) to ENV" para que funcione context.getTwilioClient().
   ========================================================================== */

// Límite simple para evitar abusos / llamadas en loop: una llamada por número
// cada MIN_GAP_MS. (Es "best effort": vive en memoria mientras la función está activa.)
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

exports.handler = async function (context, event, callback) {
  const response = new Twilio.Response();
  const origin = (event.request && event.request.headers && event.request.headers.origin) || '';
  const permitidos = String(context.ALLOWED_ORIGINS || '').split(',').map(s => s.trim()).filter(Boolean);

  response.appendHeader('Content-Type', 'application/json');
  if (permitidos.includes(origin)) {
    response.appendHeader('Access-Control-Allow-Origin', origin);
    response.appendHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
    response.appendHeader('Access-Control-Allow-Headers', 'Content-Type');
  }

  const responder = (status, body) => {
    response.setStatusCode(status);
    response.setBody(body);
    return callback(null, response);
  };

  if (!permitidos.includes(origin)) {
    return responder(403, { ok: false, error: 'Origen no permitido: ' + origin });
  }
  if (!context.TWILIO_FROM) {
    return responder(500, { ok: false, error: 'Falta configurar TWILIO_FROM en la función.' });
  }

  const to = String(event.to || '').replace(/[\s\-()]/g, '');
  if (!/^\+[1-9]\d{7,14}$/.test(to)) {
    return responder(400, { ok: false, error: 'Teléfono inválido (usá formato +5491112345678).' });
  }
  const prefijos = String(context.ALLOWED_PREFIXES || '').split(',').map(s => s.trim()).filter(Boolean);
  if (prefijos.length && !prefijos.some(p => to.startsWith(p))) {
    return responder(400, { ok: false, error: 'Ese país no está habilitado para llamadas.' });
  }

  const ahora = Date.now();
  if (ahora - (ultimas.get(to) || 0) < MIN_GAP_MS) {
    return responder(429, { ok: false, error: 'Ya se llamó a ese número hace menos de 1 minuto.' });
  }
  ultimas.set(to, ahora);

  const paciente = String(event.paciente || '').slice(0, 60);
  const motivo = String(event.motivo || 'manual');
  const detalle = String(event.detalle || '').slice(0, 120);
  const mensaje = escaparXml(textoMensaje(paciente, motivo, detalle));

  // El mensaje se repite 3 veces por si la persona tarda en atender / escuchar.
  const voz = '<Say language="es-MX" voice="Polly.Mia">' + mensaje + '</Say><Pause length="1"/>';
  const twiml = '<Response>' + voz + voz + voz + '</Response>';

  try {
    const client = context.getTwilioClient();
    const call = await client.calls.create({ to: to, from: context.TWILIO_FROM, twiml: twiml });

    let smsSid = null;
    if (String(event.sms) === '1') {
      try {
        const msg = await client.messages.create({ to: to, from: context.TWILIO_FROM, body: textoMensaje(paciente, motivo, detalle) });
        smsSid = msg.sid;
      } catch (e) {
        console.error('MOON: no se pudo mandar el SMS', e.message);
      }
    }
    return responder(200, { ok: true, callSid: call.sid, smsSid: smsSid });
  } catch (err) {
    ultimas.delete(to); // permitir reintento si falló
    console.error('MOON: error al crear la llamada', err.message);
    return responder(502, { ok: false, error: err.message, code: err.code });
  }
};

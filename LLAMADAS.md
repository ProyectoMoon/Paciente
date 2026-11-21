# Llamadas de emergencia

Cuando la pulsera detecta una **caída** o un **signo vital crítico**, la página
llama al contacto de emergencia del paciente. Hay dos formas:

| | Telegram (CallMeBot) | Teléfono (Twilio) |
|---|---|---|
| Costo | **Gratis** | Prueba gratis 30 días, después pago |
| Qué recibe el familiar | Llamada de Telegram que lee el mensaje | Llamada telefónica común |
| Estado en `index.html` | `TELEGRAM_ACTIVO = true` | `TWILIO_ACTIVO = false` |

---

# Opción 1: llamada por Telegram (gratis)

1. El **familiar** instala Telegram y tiene que tener un **nombre de usuario**
   (Ajustes > Editar perfil > Nombre de usuario, ej: `@juanperez`).
2. El familiar abre Telegram, busca **@CallMeBot_txtbot** y le manda **/start**.
   Esto se hace una sola vez y autoriza a CallMeBot a llamarlo.
3. En la página del paciente: **Editar perfil > Telegram del familiar** > `@juanperez`.
4. Probalo con el botón **"Llamar a mi contacto de emergencia"**.

Notas:
- CallMeBot es un servicio gratuito externo (no es oficial de Telegram).
- El navegador no puede leer la respuesta de CallMeBot, así que la página
  muestra "Llamando…" cuando el pedido salió. Si no llega la llamada, revisá
  que el familiar haya hecho el paso 2 y que el usuario esté bien escrito.

---

# Opción 2: llamada telefónica con Twilio (usando Vercel)

Para activarla: poné `TWILIO_ACTIVO = true` en `index.html` (aparece el campo
"Teléfono de emergencia" en el perfil) y seguí estos pasos.

La página **no** guarda las credenciales de Twilio (cualquiera podría verlas en
el código). La llamada la hace una función que corre en **Vercel** (plan gratis):

```
index.html  --POST-->  Vercel: /api/llamada-emergencia  -->  Twilio  -->  llamada al contacto
```

## 1. Twilio (cuenta de prueba gratis)
1. **Conseguí un número:** Phone Numbers > Manage > **Buy a number** (se paga con
   el crédito de prueba). Tiene que tener **Voice**.
2. **Verificá el teléfono que va a recibir las llamadas:** Phone Numbers > Manage >
   **Verified Caller IDs** > Add new. Con la cuenta de prueba, Twilio solo llama a
   números verificados, y la llamada empieza con un aviso corto en inglés.
3. Anotá el **Account SID** y el **Auth Token**. Están en la página de inicio
   de la consola; el token se ve con el ojito.

## 2. Vercel (gratis)
1. Entrá a https://vercel.com y tocá **Sign Up** > **Continue with GitHub**
   (plan **Hobby**, gratis).
2. **Add New… > Project** > importá el repo **Paciente**.
3. Antes de tocar Deploy, abrí **Environment Variables** y cargá:
   | Key | Value |
   |---|---|
   | `TWILIO_ACCOUNT_SID` | tu Account SID (empieza con `AC`) |
   | `TWILIO_AUTH_TOKEN` | tu Auth Token |
   | `TWILIO_FROM` | tu número de Twilio, ej: `+15017122661` |
   | `ALLOWED_PREFIXES` | `+54` (opcional: solo llamadas a Argentina) |
4. Tocá **Deploy**. Vercel te da una dirección tipo `https://paciente-xxxx.vercel.app`.

La página queda publicada en esa dirección y las llamadas funcionan sin tocar
nada más: la página llama a `/api/llamada-emergencia` del mismo sitio.

### ¿La página sigue publicada en otro lado (ej. GitHub Pages)?
- En Vercel agregá la variable `ALLOWED_ORIGINS` con ese dominio, sin barra
  al final, ej: `https://proyectomoon.github.io`.
- En `index.html`, cambiá `TWILIO_FUNCTION_URL` por la URL completa:
  `https://paciente-xxxx.vercel.app/api/llamada-emergencia`.

Si cambiás variables en Vercel después del deploy: Deployments > ⋯ > **Redeploy**.

## 3. Cargar el teléfono del contacto (Twilio)
En la página: **Editar perfil > Teléfono de emergencia**, con código de país.
Celular de Argentina: `+54 9` + código de área + número (ej: `+5491112345678`).

---

# Cuándo se llama (las dos opciones)
| Evento | Primera llamada | Repetición mientras siga |
|---|---|---|
| Caída | 15 s después (tiempo para tocar "Estoy bien") | cada 3 min |
| Signo vital crítico | al instante | cada 5 min |
| Botón manual | al confirmar | — |

Los tiempos se cambian en las constantes `LLAMADA_*` de `index.html`.

> Esto avisa a un familiar o cuidador. **No reemplaza** al 911 / 107.

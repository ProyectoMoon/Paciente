# Llamadas de emergencia con Twilio

Cuando la pulsera detecta una **caída** o un **signo vital crítico**, la página
llama por teléfono al contacto de emergencia del paciente y le dice un mensaje
de voz en español. También hay un botón **"Llamar a mi contacto de emergencia"**.

La página **no** guarda las credenciales de Twilio (cualquiera podría verlas en
el código). La llamada la hace una **Twilio Function**, que corre en Twilio:

```
index.html  --POST-->  Twilio Function (llamada-emergencia)  -->  llamada al contacto
```

## 1. Conseguir un número de Twilio
Console > **Phone Numbers > Manage > Buy a number** (en la cuenta trial hay
crédito de prueba). Elegí uno con capacidad **Voice**.

> **Cuenta trial:** solo puede llamar a números **verificados**
> (Phone Numbers > Manage > **Verified Caller IDs**), y la llamada empieza con
> un aviso de Twilio en inglés ("press any key…"). Al pasar a cuenta paga
> desaparecen las dos limitaciones.

## 2. Crear la Function
1. Console > **Functions and Assets > Services > Create Service** (ej: `moon`).
2. **Add > Add Function**, ruta `/llamada-emergencia`.
3. Pegá el contenido de `functions/llamada-emergencia.js`.
4. Cambiá la visibilidad de la función a **Public** (el candado al lado del nombre).
5. En **Settings > Environment Variables**:
   - Dejá tildado *"Add my Twilio Credentials (ACCOUNT_SID) and (AUTH_TOKEN) to ENV"*.
   - `TWILIO_FROM` = tu número de Twilio, ej: `+15017122661`
   - `ALLOWED_ORIGINS` = el dominio donde está publicada la página, ej:
     `https://proyectomoon.github.io` (sin barra al final; podés poner varios separados por coma)
   - `ALLOWED_PREFIXES` (opcional) = `+54` para permitir solo llamadas a Argentina.
6. **Deploy All**.
7. Copiá la URL de la función (ej: `https://moon-1234.twil.io/llamada-emergencia`).

## 3. Conectar la página
En `index.html`, buscá `TWILIO_FUNCTION_URL` y pegá esa URL.
Si querés también un SMS, poné `TWILIO_ENVIAR_SMS = true`.

## 4. Cargar el teléfono del contacto
En la página: **Editar perfil > Teléfono de emergencia**, con código de país.
Celular de Argentina: `+54 9` + código de área + número (ej: `+5491112345678`).

## Cuándo se llama
| Evento | Primera llamada | Repetición mientras siga |
|---|---|---|
| Caída | 15 s después (tiempo para tocar "Estoy bien") | cada 3 min |
| Signo vital crítico | al instante | cada 5 min |
| Botón manual | al confirmar | — |

Los tiempos se cambian en las constantes `LLAMADA_*` de `index.html`.

> Esto avisa a un familiar o cuidador. **No reemplaza** al 911 / 107: Twilio no
> permite hacer llamadas automáticas a números de emergencia.

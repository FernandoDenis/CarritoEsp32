import { Router } from 'express';
import Telemetry from '../models/Telemetry.js';
import Event from '../models/Event.js';
import { auth } from '../auth/middleware.js';


const r = Router();


// Ingesta de telemetría (desde gateway)
r.post('/telemetry', auth(), async (req, res) => {
    const doc = req.body;
    // normaliza `ts`
    if (typeof doc.ts === 'number') doc.ts = new Date(doc.ts * 1000);
    if (typeof doc.ts === 'string') doc.ts = new Date(doc.ts);
    const t = await Telemetry.create(doc);
    req.app.get('wss').broadcast(JSON.stringify({ type: 'telemetry', data: t }));
    res.json({ ok: true, id: t._id });
});


// Ingesta de eventos
r.post('/events', auth(), async (req, res) => {
    const doc = req.body;
    if (typeof doc.ts === 'number') doc.ts = new Date(doc.ts * 1000);
    if (typeof doc.ts === 'string') doc.ts = new Date(doc.ts);
    const e = await Event.create(doc);
    req.app.get('wss').broadcast(JSON.stringify({ type: 'event', data: e }));
    res.json({ ok: true, id: e._id });
});


export default r;
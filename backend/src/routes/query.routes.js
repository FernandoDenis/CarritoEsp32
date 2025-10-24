import { Router } from 'express';
import Telemetry from '../models/Telemetry.js';
import Event from '../models/Event.js';
import { auth } from '../auth/middleware.js';


const r = Router();


r.get('/telemetry', auth(), async (req, res) => {
    const { robotId, from, to, within } = req.query;
    const q = {};
    if (robotId) q.robotId = robotId;
    if (from || to) q.ts = {};
    if (from) q.ts.$gte = new Date(from);
    if (to) q.ts.$lte = new Date(to);
    if (within) {
        const [lon, lat, rad] = within.split(',').map(Number);
        q.pos = { $geoWithin: { $centerSphere: [[lon, lat], rad / 6371000] } };
    }
    const items = await Telemetry.find(q).sort({ ts: 1 }).limit(10000);
    res.json(items);
});


r.get('/events', auth(), async (req, res) => {
    const { robotId, from, to } = req.query;
    const q = {};
    if (robotId) q.robotId = robotId;
    if (from || to) q.ts = {};
    if (from) q.ts.$gte = new Date(from);
    if (to) q.ts.$lte = new Date(to);
    const items = await Event.find(q).sort({ ts: 1 }).limit(10000);
    res.json(items);
});


export default r;
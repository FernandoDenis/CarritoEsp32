import { Router } from 'express';
import Robot from '../models/Robot.js';
import { auth } from '../auth/middleware.js';


const r = Router();


r.get('/', auth(), async (req, res) => {
    const items = await Robot.find();
    res.json(items);
});


r.post('/', auth('ADMIN'), async (req, res) => {
    const it = await Robot.create(req.body);
    res.json(it);
});


export default r;
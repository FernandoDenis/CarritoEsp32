import { Router } from 'express';
import User from '../models/User.js';
import { hash, compare } from '../auth/hash.js';
import { signToken } from '../auth/jwt.js';


const r = Router();


r.post('/register', async (req, res) => {
    const { email, password, role } = req.body;
    const user = await User.create({ email, passwordHash: await hash(password), role: role || 'USER' });
    res.json({ ok: true, id: user._id });
});


r.post('/login', async (req, res) => {
    const { email, password } = req.body;
    const u = await User.findOne({ email });
    if (!u) return res.status(401).json({ error: 'Invalid credentials' });
    const ok = await compare(password, u.passwordHash);
    if (!ok) return res.status(401).json({ error: 'Invalid credentials' });
    const token = signToken({ uid: u._id.toString(), role: u.role, email: u.email });
    res.json({ token, role: u.role });
});


export default r;
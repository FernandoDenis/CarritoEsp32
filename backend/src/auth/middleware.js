import { verifyToken } from './jwt.js';
export function auth(requiredRole) {
    return (req, res, next) => {
        const h = req.headers.authorization || '';
        const token = h.startsWith('Bearer ') ? h.slice(7) : null;
        if (!token) return res.status(401).json({ error: 'No token' });
        try {
            const payload = verifyToken(token);
            if (requiredRole && payload.role !== requiredRole) {
                return res.status(403).json({ error: 'Forbidden' });
            }
            req.user = payload; next();
        } catch (err) { return res.status(401).json({ error: 'Invalid token' }); }
    };
}
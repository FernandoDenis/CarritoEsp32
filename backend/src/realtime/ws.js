import { WebSocketServer } from 'ws';
export function createWSS(server) {
    const wss = new WebSocketServer({ server });
    wss.broadcast = (msg) => {
        for (const c of wss.clients) { if (c.readyState === 1) c.send(msg); }
    };
    return wss;
}
import express from 'express';
import helmet from 'helmet';
import cors from 'cors';
import morgan from 'morgan';
import http from 'http';


import { config } from './config.js';
import { connectDB } from './db.js';
import { createWSS } from './realtime/ws.js';


import authRoutes from './routes/auth.routes.js';
import robotsRoutes from './routes/robots.routes.js';
import ingestRoutes from './routes/ingest.routes.js';
import queryRoutes from './routes/query.routes.js';
import exportRoutes from './routes/export.routes.js';


const app = express();
app.use(helmet());
app.use(cors({ origin: config.corsOrigin }));
app.use(express.json({ limit: '1mb' }));
app.use(morgan('dev'));


app.use('/auth', authRoutes);
app.use('/robots', robotsRoutes);
app.use('/ingest', ingestRoutes);
app.use('/', queryRoutes);
app.use('/', exportRoutes);


const server = http.createServer(app);
const wss = createWSS(server);
app.set('wss', wss);


await connectDB();
server.listen(config.port, () => {
    console.log(`API listening on :${config.port}`);
});
import { Router } from 'express';
import Telemetry from '../models/Telemetry.js';
import { stringify } from 'csv-stringify';
import { auth } from '../auth/middleware.js';


const r = Router();


r.get('/export/csv', auth(), async (req,res)=>{
const { robotId, from, to } = req.query;
const q = {};
if(robotId) q.robotId = robotId;
if(from || to) q.ts = {};
if(from) q.ts.$gte = new Date(from);
if(to) q.ts.$lte = new Date(to);


const cursor = Telemetry.find(q).sort({ ts: 1 }).cursor();
res.setHeader('Content-Type','text/csv');
res.setHeader('Content-Disposition','attachment; filename="telemetry.csv"');


const stringifier = stringify({ header: true, columns: ['robotId','ts','lon','lat','alt','temp','pres','spd','mode','bat'] });
stringifier.pipe(res);


for await (const d of cursor){
stringifier.write([
d.robotId,
d.ts.toISOString(),
d.pos.coordinates[0],
d.pos.coordinates[1],
d.alt ?? '', d.temp ?? '', d.pres ?? '', d.spd ?? '', d.mode ?? '', d.bat ?? ''
]);
}
stringifier.end();
});


export default r;
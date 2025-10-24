import { Schema, model } from 'mongoose';
const TelemetrySchema = new Schema({
    robotId: { type: String, index: true, required: true },
    ts: { type: Date, index: true, required: true },
    pos: {
        type: { type: String, enum: ['Point'], required: true },
        coordinates: { type: [Number], required: true } // [lon, lat]
    },
    alt: Number, temp: Number, pres: Number, spd: Number, mode: String, bat: Number
}, { timestamps: true });
TelemetrySchema.index({ pos: '2dsphere' });
TelemetrySchema.index({ robotId: 1, ts: 1 });
export default model('Telemetry', TelemetrySchema);
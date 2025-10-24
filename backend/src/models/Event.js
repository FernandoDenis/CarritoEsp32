import { Schema, model } from 'mongoose';
const EventSchema = new Schema({
    robotId: { type: String, index: true, required: true },
    ts: { type: Date, index: true, required: true },
    event: { type: String, required: true },
    extra: { type: Object }
}, { timestamps: true });
EventSchema.index({ robotId: 1, ts: 1 });
export default model('Event', EventSchema);
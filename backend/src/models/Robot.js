import { Schema, model } from 'mongoose';
const RobotSchema = new Schema({
    robotId: { type: String, required: true, unique: true, index: true },
    name: String,
    status: { type: String, enum: ['IDLE', 'RUNNING', 'OFFLINE'], default: 'OFFLINE' }
}, { timestamps: true });
export default model('Robot', RobotSchema);
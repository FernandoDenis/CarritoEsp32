import { Schema, model } from 'mongoose';
const UserSchema = new Schema({
    email: { type: String, required: true, unique: true, index: true },
    passwordHash: { type: String, required: true },
    role: { type: String, enum: ['ADMIN', 'USER'], default: 'USER' }
}, { timestamps: true });
export default model('User', UserSchema);
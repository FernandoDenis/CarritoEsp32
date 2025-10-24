import 'dotenv/config';
export const config = {
    port: process.env.PORT || 3000,
    mongo: process.env.MONGO_URI,
    jwtSecret: process.env.JWT_SECRET,
    jwtExpires: process.env.JWT_EXPIRES || '2d',
    corsOrigin: process.env.CORS_ORIGIN || '*',
};
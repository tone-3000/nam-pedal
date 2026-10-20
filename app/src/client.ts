import { T3KClient } from './tone3000-client';
import { PUBLISHABLE_KEY } from './config';

export const t3kClient = new T3KClient(PUBLISHABLE_KEY);

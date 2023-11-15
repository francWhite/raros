import express from 'express';
import { Request, Response } from 'express';
import { rosService } from './ros.service';

export const magnetRouter = express.Router();
magnetRouter.use(express.json());

magnetRouter.post('/', async (req: Request<undefined, undefined, MagnetStateModel>, res: Response) => {
  console.log('body: ', req.body);
  const payload = {
    state: req.body.active,
  };
  rosService.callService('/set_magnet', 'raros_interfaces/srv/SetMagnetState', payload, () => {});
  res.send();
});

type MagnetStateModel = {
  active: boolean;
};

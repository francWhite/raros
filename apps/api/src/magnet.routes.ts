import express from 'express';
import { Request, Response } from 'express';
import { rosService } from './ros.service';

export const magnetRouter = express.Router();
magnetRouter.use(express.json());

magnetRouter.post('/', async (req: Request<undefined, undefined, MagnetState>, res: Response) => {
  console.log(req.originalUrl, req.body);
  const payload = {
    data: req.body.active,
  };
  rosService.callService('/magnet/set_state', 'std_srvs/srv/SetBool', payload, () => {});
  res.send();
});

type MagnetState = {
  active: boolean;
};

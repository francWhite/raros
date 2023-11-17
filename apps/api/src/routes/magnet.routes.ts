import express from 'express';
import { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const magnetRouter = express.Router();
magnetRouter.use(express.json());

magnetRouter.post('/', async (req: Request<undefined, undefined, MagnetState>, res: Response) => {
  console.log(req.originalUrl, req.body);
  const requestData = {
    data: req.body.active,
  };
  rosService.callService('/raros/magnet/set_state', 'std_srvs/srv/SetBool', requestData, () => {});
  res.send();
});

type MagnetState = {
  active: boolean;
};

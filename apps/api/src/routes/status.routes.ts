import express, { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const statusRouter = express.Router();
statusRouter.use(express.json());

statusRouter.get('/', async (req: Request<undefined, undefined, undefined>, res: Response) => {
  console.log(req.originalUrl);

  rosService.callService<Status>(
    '/raros/status/get',
    'raros_interfaces/srv/GetStatus',
    {},
    (response: Status) => res.json({ ...response.status }).send(),
    (error) => res.status(500).send(error),
  );
});

type Status = {
  status: {
    is_available: boolean;
    is_moving: boolean;
    is_playing_tone: boolean;
    is_magnet_active: boolean;
    is_collision_detection_active: boolean;
  };
};

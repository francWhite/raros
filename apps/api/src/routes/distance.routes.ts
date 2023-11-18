import express, { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const distanceRouter = express.Router();
distanceRouter.use(express.json());

distanceRouter.get('/', async (req: Request<undefined, undefined, undefined>, res: Response) => {
  console.log(req.originalUrl);

  rosService.readSingleMessageFromTopic(
    '/raros/range_sensor/distance',
    'raros_interfaces/msg/Distance',
    (message: Distance) => res.json(message).send(),
    (error) => res.status(500).send(error),
  );
});

type Distance = {
  front: number;
  back: number;
};

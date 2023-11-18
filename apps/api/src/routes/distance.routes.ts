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

distanceRouter.post('/rotate-sensor', async (req: Request<undefined, undefined, Rotation>, res: Response) => {
  console.log(req.originalUrl, req.body);

  const requestData = {
    angle: req.body.angle,
  };
  rosService.callService(
    '/raros/range_sensor/rotate',
    'raros_interfaces/srv/RotateRangeSensor',
    requestData,
    () => res.send(),
    (error) => res.status(500).send(error),
  );
});

type Distance = {
  front: number;
  back: number;
};

type Rotation = {
  angle: number;
};

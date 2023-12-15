import express, { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const cameraRouter = express.Router();
cameraRouter.use(express.json());

cameraRouter.post('/rotate', async (req: Request<undefined, undefined, Rotate>, res: Response) => {
  console.log(req.originalUrl, req.body);

  rosService.callService(
    '/raros/camera/rotate',
    'raros_interfaces/srv/RotateCamera',
    req.body,
    () => res.send(),
    (error) => res.status(500).send(error),
  );
});

type Rotate = {
  angle_horizontal: number;
  angle_vertical: number;
};

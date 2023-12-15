import express, { Request, Response } from 'express';
import { rosService } from '../ros.service';

export const cameraRouter = express.Router();
cameraRouter.use(express.json());

cameraRouter.post('/capture', async (req: Request<undefined, undefined, undefined>, res: Response) => {
  console.log(req.originalUrl);

  rosService.callService(
    '/raros/camera/capture',
    'raros_interfaces/srv/CaptureImage',
    {},
    (result: CaptureResult) => res.json(result).send(),
    (error) => res.status(500).send(error),
  );
});

cameraRouter.post('/rotate', async (req: Request<undefined, undefined, Rotate>, res: Response) => {
  console.log(req.originalUrl, req.body);

  rosService.callService(
    '/raros/camera/rotate',
    'raros_interfaces/srv/RotateCamera',
    {
      angle_horizontal: req.body.angleHorizontal,
      angle_vertical: req.body.angleVertical,
    },
    () => res.send(),
    (error) => res.status(500).send(error),
  );
});

type Rotate = {
  angleHorizontal: number;
  angleVertical: number;
};

type CaptureResult = {
  image_base64: string;
};

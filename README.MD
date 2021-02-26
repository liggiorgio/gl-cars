
# OpenGL Car Physics demo
This project consists of an application written in C/C++ and OpenGL (Fedora Linux environment) where a car physics is implemented via rigid body dynamics (rather than a simpler raycast approach). I used some open source libraries to deal with loading models, textures, and the physics engine.

This application was developed as a final project for the "Real Time Graphics Programming" course at the University of Milan. A detailed report document was delivered as well, and an adaptation can be found in the Wiki section of this repo.

![Preview screenshot](https://github.com/liggiorgio/gl-cars/blob/master/wiki/main.png)

## How to use
Before compiling and starting the application, you need to install some stuff:

- OpenGL 3.3 or higher (I followed [this guide](https://en.wikibooks.org/wiki/OpenGL_Programming/Installation/Linux))

- GTK+ 3.24 for the additional GUI

- Assimp (docs page [here](https://assimp-docs.readthedocs.io/en/latest/) as 3D models loader

I actually included the `glad.h` and `stb_image.h` files in the repo: *glad* is the loader for the OpenGL drawing functions, and *[stb_image](https://github.com/nothings/stb)* is a public domain library to load/decode images (such as textures) from file/memory.

Then, you can move on and do the following:

1. Clone this repo locally
2.  [Download](https://github.com/bulletphysics/bullet3/releases) the Bullet Physics engine (version 2.83 was used during development)
3. Move the Bullet source code in the *includes* folder (so that the path to Bullet will be `./includes/bullet/*`)

To compile from terminal, use the following:

	$ g++ src/* -o App -I ./includes -lGL -lglfw -ldl -lassimp -I ./includes/bullet/ ./includes/bullet/BulletDynamics/libBulletDynamics.a ./includes/bullet/BulletCollision/libBulletCollision.a ./includes/bullet/LinearMath/libLinearMath.a `pkg-config --cflags --libs gtk+-3.0`

This is needed to link libraries all together. After compilation is finished, type the following to execute the application:

	$ ./App

## Controls
Use the arrow keys to accelerate/brake and turn left/right. Spacebar is the handbrake. Scroll the mouse wheel to adjust distance from the car, and move the mouse while holding down left click to rotate the camera around the car.

## Errors
Sometimes Assimp causes a compile-time error because of an environment variable, as discussed in [this Stack Overflow post](https://github.com/assimp/assimp/issues/1412). The quick fix to this is typing the following before compiling/executing:

	export LD_LIBRARY_PATH=/usr/local/lib

I found no other critical issues, but I can try to help if you find any.

## Credits
I went through the FANTASTIC online tutorials [LearnOpenGL](https://learnopengl.com) written by [Joey de Vries](https://joeydevries.com). Check it out if you are interested in computer graphics as well!

Also, I thank the following artists whose assets I used while making this project:
- Car model: [*Low Poly American Muscle Car*](https://opengameart.org/content/low-poly-american-muscle-car) by **T Allen Studios** on OpenGameArt.Org
- Skybox: [*clouds skybox 1*](https://opengameart.org/content/clouds-skybox-1) by **Luke.RUSTLTD** on OpenGameArt.Org
- Asphalt texture: [*Black asphalt tilling 256px*](https://opengameart.org/content/black-asphalt-tilling-256px) by **Tiziana** on OpenGameArt.Org
- Grass texture: from [*synthetic grass texture pack*](https://opengameart.org/content/synthetic-grass-texture-pack) by **para** on OpenGameArt.Org

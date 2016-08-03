The CAD vehicle (the Cadillac SRX 2010) is not from Stuntrally and was custom-
made for this simulation.

---- Ogre meshes ---------------------------------------------------------------

The original model was designed by squir
(https://www.cgtrader.com/3d-models/car/heavy-suv/cadillac-srx-2010). However,
the format was not only FBX (which is incompatible with Ogre) but also an old
version of the format.

I converted the FBX file to the latest version using Autodesk's FBX Converter.
Though the latest version is from 2013 (as of writing this) it should still work
successfully.

I then used Blender to open the FBX and followed Stuntrally's guidelines in
rearranging the model (see:
http://stuntrally.tuxfamily.org/wiki/doku.php?id=carmodeling). Firstly, I had to
combine the various meshes into a few composite meshes. There is a mesh for the
body, glass, interior (which also contains exterior meshes that wouldn't have
car paint on them), and wheels (I kept the brakes attached to the wheels). The
process of joining the meshes meant that I had to delete the textures and
materials that may have been attached to the submeshes. Despite the wiki warning
against high poly-counts, the model works fine in the simulation. The textures
needed to be redone, however.

I created UV maps for the various parts (except the body), since Stuntrally
wanted only one material per mesh. Though imperfect, the UV maps do an adequate
job resembling the original model.

After the textures were complete, I centered the car about the global origin
(0, 0, 0). For each mesh, I defined its local origin as its center of geometry
and shifted that local origin to the global origin. The wheels are an exception,
as while I redefined their local origins, I translated them so they would in
their expected positions. I scaled the model so that it would sized
approximately like the real-life version (using meters as the base unit).

To export the mesh to Ogre, I used the blender2ogre plugin (see:
http://www.ogre3d.org/tikiwiki/tiki-index.php?page=blender2ogre), following the
directions from the wiki. Note that the wheels had to be rotated 90 degrees
(specifically, its yaw) in order for it to render correctly.

The CAD materials were appended to the car.mat file in the data/materials2 dir.

---- Simulation (.car file) ----------------------------------------------------

I based my .car file off of Stuntrally's LK4 car. Some values had to be changed
based off of the model. For example, the wheel positions were derived from the
mesh (using the wheel origins). Other values were reasonably estimated but may
need tweaking. Feel free to substitute more accurate values.
# Wireframe Rendering

Plan written 2026-11-13 for drawing the edges of a contour's triangles over the mesh, in every implementation and in the interactive editor.

**Status:** complete. Steps 0 to 5 were done on 2026-11-13. Every voxelizer draws its wireframe, over its mesh or over the contour it draws straight from the GPU, and so does the interactive editor in edit mode and play mode. The README describes it. The user decided two things on 2026-11-13:
- The wireframe shows **triangle edges**, so every quad shows its diagonal. Polygon edges are out of scope (see Out of scope for how they could be added).
- There are **two looks, chosen by a setting:** lines over the shaded surface, and lines alone with the hidden ones removed.

The user gave the go-ahead on 2026-11-13, and after step 0 answered the four decisions at the end as recommended.

**Technique:** the single-pass wireframe of Bærentzen, Munk-Lund, Gjøl and Larsen (*Single-pass Wireframe Rendering*, SIGGRAPH 2006 sketch), in the common form that uses barycentric coordinates and screen-space derivatives. The paper is not in `Documents/Papers/`. The method is a few lines, and building this does not need it.

## Goal

Every voxelizer has a **Wireframe** setting, Off by default:
- **Over Surface:** the edges of the triangles are drawn as lines over the shaded mesh.
- **Lines Only:** the lines are drawn over a flat fill, so the lines behind the surface stay hidden.

The lines are anti-aliased and the same width in pixels at any distance. This works for every algorithm and every implementation, on the CPU mesh and on the GPU's direct draw, in play mode and in the interactive editor in edit mode. With the setting off, nothing is drawn differently, and every mesh is the one it is today, bit for bit.

## What exists today

- **The CPU path:** a contour's mesh goes in the `MeshFilter`, and the `MeshRenderer` draws each sub-mesh with its material. The project's opaque materials are URP Simple Lit, with smoothness 0.5.
  - **Managed meshes** (`SurfaceExtractor.createMesh`) set `vertices` and `normals`, which Unity keeps together in one stream.
  - **Burst meshes** (`BurstMesh.createMesh`) put positions in stream 0 and normals in stream 1.
  - **Index format:** indices are 16-bit up to 65,535 vertices and 32-bit past that.
  - **Split vertices:** `Voxelizer.splitVertices` gives every triangle three vertices of its own for flat shading.
- **The GPU path:** this is used with the GPU implementation and the debug view off.
  - `Voxelizer.contour` keeps the mesh on the GPU, and `Voxelizer.Update` draws it with `RenderContourDiffuse` through `Graphics.DrawProceduralIndirect`.
  - The shader fetches each vertex from the `quads` buffer by `SV_VertexID`. A quad is six indices, two triangles, and a sub-mesh index. The colours are copied from the renderer's materials.
  - The shader shades with zero smoothness, so the GPU path already looks a little different from the CPU path.
- **Drawing per camera:**
  - The voxelizer has no `[ExecuteAlways]`. In play mode, Shapes' `ImmediateModeShapeDrawer` registers `DrawShapes` on `RenderPipelineManager.beginCameraRendering` in `OnEnable`, and the voxelizer's debug view is drawn there.
  - In edit mode the interactive editor forwards that callback to `DrawShapes`, but only while the debug view is on (`InteractiveEditor.drawDebugInformation`).
  - The interactive editor contours with `drawFromGPU: false`, so in edit mode the mesh is always a CPU mesh.
- **URP:** forward rendering, depth priming off, MSAA 2×.
- **Tests:** no test renders an image today.

### Findings that shape the work

1. **Metal has no geometry shaders.** The usual way to get each triangle's barycentric coordinates, a geometry shader, does not exist on this Mac. I could not confirm that Unity compiles `SV_Barycentrics` for Metal.
2. **Shared vertices cannot carry barycentric coordinates.** Each vertex is a corner of about six triangles, and each of them needs a different coordinate there. Split vertices could carry them, but the wireframe would then only work with that setting.
3. **A non-indexed draw knows its corners.** A draw of `3 × triangles` vertices with no index buffer gives the vertex shader `SV_VertexID` = 3 × triangle + corner. The shader reads the index itself and outputs the corner's coordinate: (1,0,0), (0,1,0) or (0,0,1).
   - The `quads` buffer is already read this way, and its vertex *i* is corner *i* % 3 too, since a quad is two whole triangles.
   - The mesh's own index buffer can be read the same way, as a raw buffer.
4. **The mesh's GPU buffers can be read as raw buffers.**
   - With `GraphicsBuffer.Target.Raw` added to `Mesh.vertexBufferTarget` and `Mesh.indexBufferTarget`, `Mesh.GetVertexBuffer` and `Mesh.GetIndexBuffer` give the buffers the renderer draws from. They bind as `ByteAddressBuffer`.
   - The position's stream, offset and stride come from `GetVertexAttributeStream`, `GetVertexAttributeOffset` and `GetVertexBufferStride`. So both layouts, split vertices and the GPU's read-back meshes all work, with no copy and nothing to keep in step.
   - **Index format:** 16-bit indices are read two to a 32-bit word. Unity pads an odd count to an even one: in step 0, 99 indices gave a buffer of 100 × 2 bytes. So the last word is always inside the buffer.
   - **Disposal:** each `GetVertexBuffer` and `GetIndexBuffer` returns a new wrapper that has to be disposed.
   - **Rebuilt buffers:** setting the buffer targets rebuilds the buffers. Changing the sub-mesh count does not, and neither do `RecalculateBounds` and `UploadMeshData` (step 0). So wrappers taken when a mesh is shown stay valid when the interactive editor pads its sub-meshes after a contour (`InteractiveEditor.cs:594`).
5. **Drawing the wireframe *instead of* the renderer would cost picking and the materials' look.**
   - Unity picks the contour's renderer by depth, and the interactive editor relies on that: a click on the mesh in front of an outline selects the mesh.
   - A replacement draw would also replace the Simple Lit shading with a copy, and the shadows with another shadow pass.
   - Drawn *over* the renderer, the wireframe leaves all of that as it is.
6. **An overlay's depth cannot be relied on to match the renderer's bit for bit.** Simple Lit's vertex shader and the overlay's are compiled separately, and the compiler arranges float operations as it sees fit (Shape Rotation, finding 7).
   - **Measured in step 0, it does match.** With no offset at all, the overlay z-fought with Simple Lit at no pixel in any of 12 views, moved, turned and scaled included (Measurements).
   - **A slope-scaled offset lets lines through.** With `Offset -1, -1`, hidden lines showed at up to 18 pixels a view. They were along the concave crease where a ball meets a cube, and at grazing angles.
   - **So the overlay is drawn with `Offset 0, -1`, after the opaque objects:** one unit of depth and no slope. In step 0 it leaked nothing and z-fought nowhere. The one unit allows for a compiler that does arrange the two shaders' arithmetic differently.
7. **A line on an edge between two triangles is drawn half by each.** Each triangle draws the part of the line on its own side, so the two halves meet at the edge. On an edge with one triangle, at an open boundary, the line is half as wide. On a silhouette the other triangle faces away, so the line is half as wide there too. That is the usual look of this technique, and it is accepted here.
8. **Dense meshes fill up with lines.** With the grid about 600 pixels across, a cell is about 4.7 pixels at 128³ and 2.3 at 256³, and each cell holds about two triangles. Lines of 1 pixel then cover most of the surface. The setting still works there, and the lines can be seen by zooming in. Fading the lines by triangle size is out of scope.
9. **The edit mode draw has to go through the interactive editor,** as the debug view's does (Drawing per camera, above). Its forwarding has to run while the wireframe is on as well as while the debug view is.

## Design

### Settings

On every voxelizer, under a **Wireframe** header, beside the debug settings:

| Setting | Default | What it does |
|---|---|---|
| Wireframe | Off | Off, Over Surface or Lines Only. |
| Wireframe Line Width | 1 px | The width of the lines in pixels. At least 0.5, as the debug line thickness is. |
| Wireframe Line Color | black | The colour of the lines over the surface. Its alpha blends them. |
| Wireframe Fill Color | black | The fill under Lines Only, black to match the void skybox. Lines Only draws each sub-mesh's lines in its material's colour (decision 3). |

The names are as the inspector shows them, from the fields `wireframe`, `wireframeLineWidth`, `wireframeLineColor` and `wireframeFillColor`.

They change only what is drawn. They are not part of the interactive editor's signature, so changing them does not contour again. `InteractiveEditor.select` carries them over to the next voxelizer, as it does Vertex Normals.

### The shader

`Assets/Resources/Shaders/RenderContourWireframe.shader` has one unlit pass, in the transparent queue. It uses `ZTest LEqual`, `ZWrite Off`, `Offset 0, -1`, alpha blending, and culls back faces as the materials do. Two keyword variants choose where the vertices come from:
- **Mesh:**
  - **Buffers:** `ByteAddressBuffer`s for the position stream and the index buffer.
  - **Constants:** the position's offset and stride, whether the indices are 16-bit, and the sub-mesh's first index and base vertex.
  - **Transform:** the voxelizer's `localToWorld`, passed as `RenderContourDiffuse` takes it.
- **Quads:** the GPU path's `vertices` and `quads` buffers, with the `Quad` layout declared as `RenderContourDiffuse` declares it. `RenderContourDiffuse` itself is left untouched, so the GPU path's shading stays exactly as it is.

What the shader does:
- **Vertex stage:** the vertex shader reads the index for `SV_VertexID`, fetches and transforms the position, and outputs the corner's barycentric coordinate. With the quads variant it also outputs the triangle's sub-mesh index, without interpolation.
- **Fragment stage:** the fragment shader takes `d = b / fwidth( b )`, the distance in pixels to each of the triangle's edges, and the smallest of the three. Over a pixel, the line's coverage goes from 1 inside half the width to 0 outside it.
  - **Over Surface** outputs the line colour with the coverage times its alpha, and discards pixels the line does not cover.
  - **Lines Only** outputs `lerp( fill, line, coverage )` with alpha 1, so it covers the shaded surface beneath.
- **Line colour in Lines Only:** each sub-mesh's material colour, from a colour array indexed by sub-mesh, as `RenderContourDiffuse` has ten.

### Drawing

`Wireframe` (`Assets/Scripts/Wireframe.cs`, from step 1) holds the material, a property block for each sub-mesh and the mesh's buffers. Its `draw( camera, mesh, localToWorld, layer, style, materials )` draws a mesh for one camera.
- **Materials:** `materials` are the renderer's. Lines Only takes each sub-mesh's colour from its material. A sub-mesh the renderer has no material for, or a null one, gets no lines, since the renderer does not draw it either.
- **Empty meshes:** a mesh without vertices or indices has no buffers and is passed over. That is the mesh of an empty contour. The voxelizer keeps one `Wireframe` and gains `drawWireframe( Camera )`, which calls it and skips preview and reflection cameras, as the voxelizer's other drawing does.
- **CPU mesh:** it draws the mesh this voxelizer showed, while that mesh is still in the `MeshFilter` and the `MeshRenderer` is enabled.
  - **Draws:** one `Graphics.RenderPrimitives` call per non-empty sub-mesh, with `3 × triangles` vertices and the sub-mesh's first index, base vertex and colour, through `RenderParams.camera` and a property block.
  - **Buffers:** the raw targets are added to the mesh only when the wireframe is on. Setting the targets rebuilds the buffers, so the wrappers are taken after it. They are kept for the mesh, and are disposed and taken again when the voxelizer shows another mesh. Nothing else has to take them again (finding 4).
- **GPU direct draw:** `Wireframe.drawQuads` draws with the quads variant, over the voxelization's existing arguments buffer.
  - **The call:** it is `Graphics.DrawProceduralIndirect`, as the contour's own draw is: `RenderPrimitivesIndirect` takes a `GraphicsBuffer`, and the GPU's arguments are a `ComputeBuffer`.
  - **Transform:** the lines use the matrix the contour is drawn with, the one it had when it was contoured.
  - **Every quad gets lines,** since the contour draws every quad whatever the renderer's materials.
  - **Released buffers draw nothing.** Drawing from them crashes Unity (step 3).
- **When it runs:** `DrawShapes` calls it first, then draws the debug view as now. In play mode Shapes calls `DrawShapes` for each camera; in edit mode the interactive editor forwards to it while the wireframe or the debug view is on.
- **The material:** the voxelizer makes its `Wireframe` the first time the wireframe is on, and disposes of it with itself. The material in it has `HideFlags.DontSave`, as the GPU path's material does.

### Interactive editor

- **Forwarding:** the forwarding condition gains the wireframe (finding 9), and the method that forwards is now `drawDebugViewAndWireframe`.
  - **Both modes:** the editor disables its voxelizers in play mode, so its forwarding draws their lines in play mode as well as in edit mode.
  - **Nothing twice:** a voxelizer draws for itself only once its `OnEnable` has run, so nothing is drawn twice.
- **Carry-over:** `select` carries the four settings over.
- **Inspector:** the selected voxelizer's inspector shows them below the editor, as it shows its other settings.

### Example scenes

Nothing changes in them: the wireframe is off until it is turned on in a voxelizer's inspector.

## Tests

A new edit mode helper renders a camera into a render texture with `RenderPipeline.SubmitRenderRequest`, the render pipeline's own way, and reads the pixels back. In step 0, it and `Camera.Render` each fired `beginCameraRendering` once and gave the same image. The tests:

- **A cube, head on:** a Uniform Burst cube in a 16³ grid that fills a 512-pixel orthographic view, so each cell is 32 pixels.
  - **Off:** the image is the same as with no wireframe drawn at all.
  - **Over Surface:** points a quarter, half and three quarters along each projected edge that two front triangles share are the line colour. The centres of the triangles are as with Off.
  - **Lines Only:** the points on edges are the material's colour, and the triangle centres are the fill colour.
  - **Working out the samples:** the edges and centres are projected on the CPU from the mesh and the camera.
  - **Line width:** the lines are drawn 3 pixels wide, so the pixel of any point on an edge is wholly line.
  - **Colours:** they are not 0 or 1 in every channel, so a colour taken to linear space twice, or not at all, shows.
- **Hidden lines:** back faces are culled, so lines behind the surface are tested with two cubes, one partly behind the other.
  - **Under both looks,** the points on the rear cube's edges that the front cube hides show the front cube, and those in view show lines.
  - **The cubes are drawn in perspective.** The vertices of both front faces are at the centres of their cells, so seen straight on, the rear cube's lines would fall on the front cube's.
- **Layouts:** the same triangles as a managed mesh (one stream) and a Burst mesh (two streams), each with 16-bit and 32-bit indices, give the same image bit for bit.
  - **Index counts:** the first sub-mesh is given an odd number of indices, so the second starts halfway into a word of 16-bit indices.
  - **Vertices:** shared and split vertices give the same image too.
- **Sub-meshes:** with two materials, Lines Only draws each sub-mesh's lines in its material's colour. A sub-mesh the renderer has no material for gets no lines.
- **Empty meshes:** a mesh without vertices, a mesh without triangles, and a voxelizer whose shapes are all outside the grid draw nothing, and log nothing.
- **The voxelizer** draws the lines of the contour it shows, the same image as `Wireframe` drawn over that mesh by the test. It draws none:
  - with the wireframe off;
  - over a mesh it did not make;
  - while its renderer is off;
  - on a layer the camera does not see.
  
  After another contour its lines are those of the new mesh, and after `release` they are the same as before.
- **GPU:** the lines the quads variant draws from the GPU's buffers are the same image, bit for bit, as those the mesh variant draws over the mesh read back from those buffers.
  - **Cases:** Uniform and Occupancy, shared and split vertices, and both looks.
  - **The surface:** `Update` draws the GPU's contour, and only in play mode, so in the tests a mesh renderer of the read-back mesh gives the surface and its depth.
  - **Off and after `release`,** nothing is drawn.
  - **Why not against the CPU's mesh, as first planned:** a comparison with the CPU's mesh would also catch the contourers' unit or two in the last place, which the GPU tests already cover.
- **Meshes unchanged:** with the wireframe on and off, the mesh's vertices and indices are the same. The `Start` hashes of the example scenes are identical to the step 0 baseline.
- **Interactive editor:**
  - `select` carries the settings over, in the two carry-over tests that already were.
  - Changing them does not change the signature: three more cases of `theSignatureStaysTheSameWith`.
  - The selected voxelizer's wireframe is drawn, through the editor, by a camera of the test's own.
  - The tests make hidden objects and use no undo.
- **The tests catch a mistake:** with the corner taken from the wrong slot, or with the 16-bit indices read as 32-bit, the image tests fail.
- **By eye:** scene view captures at a grazing angle, at a crease, at a silhouette, and at 128³ and 256³, for findings 6 to 8.

## Steps

Each step compiles, passes every test, and is reviewed before the next one starts.

0. **Baseline and checks (small).**
   - Both suites green, and the `Start` hashes recorded.
   - Then, with throwaway code in the scratchpad:
     - raw mesh buffers bind and draw on Metal, for both layouts and both index formats, including an odd number of 16-bit indices;
     - whether changing the sub-mesh count rebuilds the index buffer;
     - a `RenderPrimitives` call from `beginCameraRendering` shows in the scene and game views, in edit mode and in play mode;
     - the render texture helper fires `beginCameraRendering`;
     - the depth offset hides z-fighting at a grazing angle without lines showing through at a crease.
   - The answers are written here before step 1.
   - **Done 2026-11-13:**
     - **Suites:** edit mode 1766 of 1766, play mode 21 of 21.
     - **`Start` hashes:** the probe now also covers Occupancy, Burst and GPU, for 480 configurations. The 416 lines of the other three algorithms are identical to Shape Rotation's last baseline, and the two Occupancy GPU runs agree. These 480 are the baseline from here on.
     - **Raw buffers:** a 4 × 4 grid in a second sub-mesh that starts at index 3, in the middle of a word of 16-bit indices. It was drawn from both layouts, both index formats, 32 and 31 triangles (99 and 96 indices), and both ways of rendering into a texture. All 16 cases are right, and each triangle count gives one image. Odd 16-bit counts are padded (finding 4).
     - **Rebuilt buffers:** only setting the targets rebuilds them (finding 4).
     - **Views:** drawn over the interactive editor's contour (60,828 vertices, Burst layout, split vertices, 16-bit indices), the overlay shows:
       - in the scene view, both rendered on request and as Unity draws the window;
       - in the game view in edit mode;
       - in the game view in play mode.
     - **Render texture helper:** `SubmitRenderRequest` and `Camera.Render` both fire `beginCameraRendering` once (Tests).
     - **Depth offset:** `Offset 0, -1` instead of `-1, -1` (finding 6, Measurements).
     - **Clean-up:** the probe (`WireframeProbe.cs` and `.shader`, in the session's scratchpad) compiled its shader in memory and put nothing in `Assets`. Afterwards the open scene had its four roots and was not modified, and the console had no new errors or warnings.
1. **Shader and render tests (small).** `RenderContourWireframe.shader` with the mesh variant, the render texture helper, and the cube and layout tests, drawn by the test itself. **Done 2026-11-13:**
   - **Built:**
     - `RenderContourWireframe.shader`, reading from the mesh only. The quads come in step 3.
     - `Wireframe` (Drawing).
     - `OffscreenRender`, the render texture helper, in the edit mode tests.
     - `WireframeTest`, 13 tests (Tests).
   - **Drawn by `Wireframe`, not by the tests.** The drawing became a class of its own, so the tests draw with the code the voxelizer will call in step 2, not with a copy.
   - **Samples per check,** each about twice its threshold:
     - on the cube: 675 points on edges and 162 triangle centres;
     - on the two cubes: 237 points on the front cube, 34 hidden points on the rear cube and 103 in view.
     - **Why three points per edge:** midpoints alone gave only 7 hidden points, since most of the rear cube's midpoints are within 4 pixels of the front cube's lines.
   - **Colour spaces:** `MaterialPropertyBlock.SetColor` converts a colour to linear space and `SetVectorArray` does not, so `Wireframe` converts the array itself. The tests' colours show both.
   - **The tests catch a mistake.** Each mistake below was made on purpose, then undone:
     - the corner taken from the mesh's vertex index instead of the draw's: 7 tests failed;
     - 16-bit indices read as 32-bit: 10 failed;
     - the halves of a word of 16-bit indices swapped: 6 failed;
     - `ZTest Always`: 3 failed, catching all 34 hidden points;
     - the material colours not taken to linear space: 3 failed, those of Lines Only.
   - **Verification:**
     - edit mode 1779 of 1779 and play mode 21 of 21;
     - the 480 `Start` hashes identical to step 0's;
     - the open scene clean, and no errors or warnings in the console.
2. **Voxelizer (medium).** The settings, `drawWireframe` for CPU meshes, the mesh buffers and their lifetime, sub-mesh colours, `DrawShapes`. The sub-mesh and unchanged-mesh tests, and play mode in an example scene. **Done 2026-11-13:**
   - **Built:**
     - The four settings (Settings).
     - `Voxelizer.drawWireframe`, which `DrawShapes` calls first.
     - `Voxelizer.release`, which now lets go of the `Wireframe`. The two implementations that override it, Uniform's and Occupancy's GPU code, call it.
     - `Wireframe.draw` takes the renderer's materials instead of colours, so a null material gets no lines.
   - **Tests:**
     - 8 voxelizer tests and 5 empty-mesh tests in `WireframeTest` (Tests), and the four defaults in `VoxelizerDefaultsTest`.
     - The voxelizer's images are checked against `Wireframe` drawn by the test over the same mesh, which step 1's tests hold to known pixels.
   - **The play check found a bug.**
     - **The bug:** an empty contour's mesh has a position attribute but no vertices, and so no buffers. Each frame and camera, asking for them threw.
     - **The fix:** `Wireframe` now passes over meshes without vertices or indices. Two of the three new tests failed before the fix. The third, a mesh with no position attribute at all, was already passed over.
   - **The tests catch a mistake.** Each mistake below was made on purpose, then undone:
     - lines over any mesh in the filter: 1 test failed;
     - without the renderer, active and layer checks: 1 failed, the renderer's;
     - `DrawShapes` not drawing the wireframe: 3 failed. The release test passed as well, since it drew nothing before or after; it now also checks that something was drawn;
     - `release` disposing without letting go: 1 failed.
   - **Unity culls the lines by the camera's culling mask itself.** With the voxelizer's own layer check removed, the layer test still passed. The check stays, as a way out before the lines are set up.
   - **Play mode,** in the interactive editor's scene:
     - **A voxelizer added in play mode** contoured in `Start`, as an example scene's do, and drew Lines Only in the game view through Shapes' `DrawShapes`. Its first contour was empty, and with the fix 2,000 frames of it logged nothing.
     - **The editor's own voxelizer** is disabled in play mode, where the editor contours, so its lines wait for step 4's forwarding (finding 9).
   - **Verification:**
     - edit mode 1792 of 1792 and play mode 21 of 21;
     - the 480 `Start` hashes identical to step 0's, before and after the empty-mesh fix;
     - the open scene clean, and no errors or warnings in the console.
3. **GPU direct draw (small).** The quads variant and the indirect draw. The GPU against CPU test. **Done 2026-11-13:**
   - **Built:**
     - The `CONTOUR_QUADS` variant of the shader. It reads `RenderContourDiffuse`'s `Quad` layout, and passes each quad's sub-mesh to the fragment shader.
     - `Wireframe.drawQuads`.
     - The voxelizer keeps the GPU's vertex and quad buffers while it draws from them, and `drawWireframe` draws their lines (Drawing).
   - **Tests:** 11 in `WireframeTest` (Tests).
     - **All 8 image comparisons are exact,** for Uniform and Occupancy, shared and split vertices, and both looks.
     - **They are against the read-back mesh** instead of the CPU's mesh (Tests).
   - **The tests catch a mistake.** Each mistake below was made on purpose, then undone:
     - the wrong slot of a quad: 8 tests failed;
     - every quad in sub-mesh 0: 4 failed, those of Lines Only;
     - the voxelizer not drawing the GPU's lines: 8 failed;
     - `drawQuads` drawing from released buffers: **Unity crashed**, in `DrawProceduralIndirect`. The check is what the release test stands for, and it stays.
   - **The crash,** at 17:04:
     - **What was lost:** it took the editor down mid-run. The user's scene edits had been saved at 16:59, and the restored code was what the new session compiled.
     - **What it left behind:** a `mono_crash` dump in the project root.
     - **What else it shows:** the contour's own draw in `Voxelizer.Update` would crash the same way if `release` were called while a contour is drawn from the GPU. Nothing does that: `OnDestroy` stops the direct draw before it releases.
   - **Play mode:** two GPU voxelizers added in play mode drew from the GPU, one with lines over its surface and one with lines only. The console stayed empty.
   - **Verification:**
     - edit mode 1803 of 1803 and play mode 21 of 21;
     - the 480 `Start` hashes identical to step 0's;
     - the open scene clean, and no errors or warnings in the console.
4. **Interactive editor (small).** Forwarding, carry-over, the inspector. Its tests, and a live check in the editor scene. **Done 2026-11-13:**
   - **Built:** the forwarding and the carry-over (Interactive editor). The inspector needed nothing: the voxelizer's default inspector shows the four settings under their header, between Vertex Normals and the debug settings.
   - **Tests:** five in `InteractiveEditorTest` (Tests).
     - **The carry-over tests** set and check the four settings, in every direction they already covered.
     - **A sixth test was dropped.** It held that an unselected voxelizer draws no lines, but it could not fail: such a voxelizer never has a contour of its own on show.
   - **The tests catch a mistake.** Each mistake below was made on purpose, then undone:
     - forwarding only for the debug view: 2 failed;
     - `select` without the wireframe settings: 5 failed.
   - **Live, in the open editor scene:**
     - **Edit mode:** Over Surface and Lines Only showed over the editor's contour in the scene view, with the shape outlines and the selection outline. Lines Only drew each material's lines in its colour.
     - **No changes left behind:** neither setting contoured again, the scene stayed unmodified, and the setting and the selection were put back afterwards.
     - **Play mode:** the editor's own contour got its lines, which step 2 could not show, and the console stayed empty.
   - **Verification:**
     - edit mode 1808 of 1808 and play mode 21 of 21;
     - the 480 `Start` hashes identical to step 0's;
     - the open scene clean.
5. **Measurements and documentation (small).**
   - Frame time with each look, at 128³ and 256³, on Burst and on the GPU, with Unity in front.
   - The README: the setting, and its place in the interactive editor section.
   - This plan marked complete.
   - **Done 2026-11-13:**
     - **Frame times:** under Measurements.
     - **The README:**
       - a **Wireframe** section after Shapes: the four settings, how the lines are drawn, what they cost, and dense meshes;
       - the wireframe among the settings the interactive editor carries over;
       - `Wireframe.cs` and its shader in the code layout.

## Measurements

### Frame time (step 5, 2026-11-13)

**Setup:**
- **Model:** Uniform Dual Contouring of the three balls, with `opaque_white` and lines 1 pixel wide.
- **Camera:** an offscreen camera at 1920 × 1080 with 2× MSAA, as the URP asset has it.
- **Timing:** frames go through `RenderPipeline.SubmitRenderRequest`, ten at a time, with a wait for the GPU after each ten. The time per frame is the median over 30 such batches, the three looks taken in turn. Unity was in front throughout.
- **The GPU:** the surface is the mesh read back, since `Update` draws the GPU's own contour only in play mode. The lines are drawn from the GPU's buffers.

Three runs each, in ms:

| Implementation | Cells | Triangles | Off | Over Surface | Lines Only |
|---|---|---|---|---|---|
| Burst | 128³ | 104,500 | 1.03, 0.99, 0.83 | 1.08, 1.14, 1.39 | 1.08, 1.13, 1.41 |
| Burst | 256³ | 417,876 | 1.06, 1.05, 1.04 | 1.13, 1.16, 1.14 | 1.15, 1.15, 1.15 |
| GPU | 128³ | 104,500 | 1.02, 0.99, 1.02 | 1.07, 1.04, 1.07 | 1.07, 1.09, 1.06 |
| GPU | 256³ | 417,876 | 1.03, 1.09, 1.10 | 1.23, 1.32, 1.30 | 1.22, 1.34, 1.33 |

**What they show:**
- **A frame is about 1 ms at either size,** which is mostly the render request's own cost.
- **From Burst,** the lines add 0.05 to 0.15 ms. One run of 128³ added 0.56 ms, and the other two did not repeat it.
- **From the GPU,** they add 0.05 ms at 128³ and 0.2 to 0.25 ms at 256³. Why the quads cost more than the mesh at 256³ was not looked into.
- **The two looks cost the same,** within the runs' spread.
- **A first attempt waited for the GPU after every frame.** Its Off times varied by up to 0.5 ms from run to run, more than the lines add, and it was replaced.

### Depth offset (step 0, 2026-11-13)

**Setup:**
- **Models:** Uniform Burst contours of the `Start` probe's models, drawn by a `MeshRenderer` with `opaque_white` (Simple Lit).
- **Images:** 512 × 512, without MSAA, with the overlay under Lines Only at a line width of 1 pixel.
- **The reference:** the overlay drawn alone with depth writes, which removes hidden lines exactly.

**What the table counts:** inside the silhouette, the pixels where the overlay over the renderer differs from the reference.
- No pixel showed the renderer's shading (z-fighting) at any offset.
- Every difference counted is a hidden line or fill showing through.

**The views:**
- **General:** 35° above, from the side.
- **Grazing:** 3° over the cubes' top faces.
- **Edge on:** almost along the cubes' top front edge, both faces at about 3°.

| Model | Cells | Transform | Silhouette (px), general / grazing / edge on | `-1, -1` | `0, -1` | `0, 0` |
|---|---|---|---|---|---|---|
| Cubes and sphere | 32³ | none | 49,545 / 44,888 / 31,798 | 18 / 3 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Cubes and sphere | 32³ | moved, turned, scaled 2.3 | 49,545 / 44,888 / 31,798 | 18 / 3 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Cubes and sphere | 64³ | moved, turned, scaled 2.3 | 49,611 / 44,914 / 31,793 | 2 / 6 / 2 | 0 / 0 / 0 | 0 / 0 / 0 |
| Sphere | 64³ | none | 28,963 / 28,844 / 29,445 | 0 / 2 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

## Out of scope

- **Polygon edges.** Decided on 2026-11-13 for triangle edges.
  - **How they could be added:** each triangle would carry three bits that say which of its edges are polygon edges, set where each implementation emits polygons. Pairs of triangles cannot be trusted to be quads:
    - Manifold's clustering drops or collapses some triangles;
    - Occupancy can split a polygon into four;
    - the planned simplifier gives plain triangles.
  - The GPU's quads would not need the bits: the diagonal is the edge both triangles share.
- **A see-through wireframe,** with the lines behind the surface shown.
- **Fading lines** where triangles are only a few pixels across (finding 8).
- **Shadows under Lines Only.** The renderer still draws the shaded surface beneath the fill, and that surface still casts shadows.
- **Transparent materials** on the voxelizer, under which the overlay's order is not defined.
- **The scene view's own wireframe draw modes,** which already work on CPU meshes and are not changed.

## Decisions

1. **Draw over the renderer, not instead of it?** Findings 5 and 6. An overlay keeps picking, the selection outline, the materials' shading and the shadows as they are, at the cost of a second draw of the geometry and a depth offset of one unit, which leaked nothing in step 0. A replacement draw has no offset, but needs its own shading, shadows and picking. Recommended: the overlay.
2. **Read the mesh's own GPU buffers?** Finding 4. In chat I suggested uploading copies of the positions and indices. Reading the mesh's buffers turned out simpler: the layout comes from the mesh, so nothing is copied after each contour and nothing can fall out of step with what the renderer draws. Recommended: the mesh's buffers, which step 0 drew from in every layout and index format.
3. **Lines Only in the materials' colours over a black fill?** This shows which material each surface is when the shading is gone. The other way uses the line colour on the fill, which needs a line colour that shows on the fill. Recommended: the materials' colours.
4. **Settings on every voxelizer?** They sit with Vertex Normals and the debug view, and the interactive editor carries them over. The other way is a separate Wireframe component on the object, which keeps the voxelizer smaller but has to find the voxelizer that showed the mesh. Recommended: on the voxelizer.

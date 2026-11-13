# Wireframe Rendering

Plan written 2026-11-13 for drawing the edges of a contour's triangles over the mesh, in every implementation and in the interactive editor.

**Status:** draft. Nothing is built and nothing was measured for it. The user decided two things on 2026-11-13:
- The wireframe shows **triangle edges**, so every quad shows its diagonal. Polygon edges are out of scope (see Out of scope for how they could be added).
- There are **two looks, chosen by a setting:** lines over the shaded surface, and lines alone with the hidden ones removed.

The four decisions at the end are open, and step 0 waits for them and for the go-ahead.

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
   - **Index format:** 16-bit indices are read two to a 32-bit word. Step 0 checks what is past the last index when the count of 16-bit indices is odd.
   - **Disposal:** each `GetVertexBuffer` and `GetIndexBuffer` returns a new wrapper that has to be disposed.
   - **Rebuilt buffers:** the buffers can be rebuilt when the mesh changes. The interactive editor changes the shown mesh's sub-mesh count after a contour (`InteractiveEditor.cs:594`), and step 0 checks whether that rebuilds the index buffer.
5. **Drawing the wireframe *instead of* the renderer would cost picking and the materials' look.**
   - Unity picks the contour's renderer by depth, and the interactive editor relies on that: a click on the mesh in front of an outline selects the mesh.
   - A replacement draw would also replace the Simple Lit shading with a copy, and the shadows with another shadow pass.
   - Drawn *over* the renderer, the wireframe leaves all of that as it is.
6. **An overlay's depth cannot match the renderer's bit for bit.** Simple Lit's vertex shader and the overlay's are compiled separately, and the compiler arranges float operations as it sees fit (Shape Rotation, finding 7).
   - The overlay is drawn with a small depth offset (`Offset -1, -1`) and `ZTest LEqual`, after the opaque objects.
   - The cost is that a line can show through a surface that is within the offset in front of it, as at a sharp crease seen edge on. Step 0 looks for this.
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
| Wireframe Line Colour | black | The colour of the lines over the surface. Its alpha blends them. |
| Wireframe Fill Colour | black | The fill under Lines Only, black to match the void skybox. Lines Only draws each sub-mesh's lines in its material's colour (decision 3). |

They change only what is drawn. They are not part of the interactive editor's signature, so changing them does not contour again. `InteractiveEditor.select` carries them over to the next voxelizer, as it does Vertex Normals.

### The shader

`Assets/Resources/Shaders/RenderContourWireframe.shader` has one unlit pass, in the transparent queue. It uses `ZTest LEqual`, `ZWrite Off`, `Offset -1, -1`, alpha blending, and culls back faces as the materials do. Two keyword variants choose where the vertices come from:
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

The voxelizer gains `drawWireframe( Camera )`. It skips preview and reflection cameras, as the voxelizer's other drawing does.
- **CPU mesh:** it draws the mesh this voxelizer showed, while that mesh is still in the `MeshFilter` and the `MeshRenderer` is enabled.
  - **Draws:** one `Graphics.RenderPrimitives` call per non-empty sub-mesh, with `3 × triangles` vertices and the sub-mesh's first index, base vertex and colour, through `RenderParams.camera` and a property block.
  - **Buffers:** the raw targets are added to the mesh only when the wireframe is on. The wrappers are kept for the mesh, and are disposed and taken again when the voxelizer shows another mesh. Whether anything else has to take them again is step 0's question (finding 4).
- **GPU direct draw:** one `Graphics.RenderPrimitivesIndirect` call over the voxelization's existing arguments buffer, with the quads variant.
- **When it runs:** `DrawShapes` calls it first, then draws the debug view as now. In play mode Shapes calls `DrawShapes` for each camera; in edit mode the interactive editor forwards to it while the wireframe or the debug view is on.
- **The material** is made once per voxelizer with `HideFlags.DontSave`, as the GPU path's material is, and destroyed with it.

### Interactive editor

- **Forwarding:** the forwarding condition gains the wireframe (finding 9).
- **Carry-over:** `select` carries the four settings over.
- **Inspector:** the selected voxelizer's inspector shows them below the editor, as it shows its other settings.

### Example scenes

Nothing changes in them: the wireframe is off until it is turned on in a voxelizer's inspector.

## Tests

A new edit mode helper renders a camera into a render texture and reads the pixels back. Step 0 picks between `Camera.Render` and `RenderPipeline.SubmitRenderRequest`, and has to confirm that `beginCameraRendering` fires for it. The tests:

- **A cube, head on:** a Uniform Burst cube in a 16³ grid that fills a 512-pixel view, so each cell is about 32 pixels.
  - **Off:** the image is the image of today.
  - **Over Surface:** the pixels at the midpoints of the projected triangle edges are the line colour. The pixels at the centres of the triangles are as with Off.
  - **Lines Only:** the edge midpoints are the material's colour and the triangle centres are the fill colour.
  - **Hidden lines:** the back faces' edges do not show anywhere.
  - The projected edges and centres are worked out on the CPU from the mesh and the camera.
- **Layouts:** the same triangles as a managed mesh (one stream) and a Burst mesh (two streams), each with 16-bit and 32-bit indices, give the same image bit for bit. So do shared and split vertices. An odd number of 16-bit indices is one of the cases.
- **Sub-meshes:** a model with two materials draws each sub-mesh's lines in its material's colour under Lines Only.
- **GPU against CPU:** the GPU's direct draw and the CPU mesh of the same contour under Lines Only differ in only a few edge pixels. GPU positions are a unit or two in the last place off the CPU's, which can move a pixel across an edge.
- **Meshes unchanged:** with the wireframe on and off, the mesh's vertices and indices are the same. The `Start` hashes of the example scenes are identical to the step 0 baseline.
- **Interactive editor:**
  - `select` carries the settings over.
  - Changing them does not change the signature.
  - The wireframe is drawn in edit mode.
  - Tests revert their own undo groups.
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
1. **Shader and render tests (small).** `RenderContourWireframe.shader` with the mesh variant, the render texture helper, and the cube and layout tests, drawn by the test itself.
2. **Voxelizer (medium).** The settings, `drawWireframe` for CPU meshes, the mesh buffers and their lifetime, sub-mesh colours, `DrawShapes`. The sub-mesh and unchanged-mesh tests, and play mode in an example scene.
3. **GPU direct draw (small).** The quads variant and the indirect draw. The GPU against CPU test.
4. **Interactive editor (small).** Forwarding, carry-over, the inspector. Its tests, and a live check in the editor scene.
5. **Measurements and documentation (small).**
   - Frame time with each look, at 128³ and 256³, on Burst and on the GPU, with Unity in front.
   - The README: the setting, and its place in the interactive editor section.
   - This plan marked complete.

## Measurements

None yet.

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

1. **Draw over the renderer, not instead of it?** Findings 5 and 6. An overlay keeps picking, the selection outline, the materials' shading and the shadows as they are, at the cost of a depth offset and a second draw of the geometry. A replacement draw has no offset, but needs its own shading, shadows and picking. Recommended: the overlay.
2. **Read the mesh's own GPU buffers?** Finding 4. In chat I suggested uploading copies of the positions and indices. Reading the mesh's buffers turned out simpler: the layout comes from the mesh, so nothing is copied after each contour and nothing can fall out of step with what the renderer draws. Recommended: the mesh's buffers, confirmed by step 0.
3. **Lines Only in the materials' colours over a black fill?** This shows which material each surface is when the shading is gone. The other way uses the line colour on the fill, which needs a line colour that shows on the fill. Recommended: the materials' colours.
4. **Settings on every voxelizer?** They sit with Vertex Normals and the debug view, and the interactive editor carries them over. The other way is a separate Wireframe component on the object, which keeps the voxelizer smaller but has to find the voxelizer that showed the mesh. Recommended: on the voxelizer.

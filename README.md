# runelite-SPIRV
runelite's collected GLSL shaders to be used in creating a SPIR-V version for future GPU use.

Toolchain: https://github.com/google/shaderc

Toolchain documentation: https://github.com/google/shaderc/tree/master/glslc

--Structure of the Code--
Scale: Holds the coding for the UI Scaling of the models.

Uneeded: Files that held code for the Shader units; the code being small functions from things like measuring distance, color conversions, camera angles, etc.

Because SPIR-V/GLSL/GLSLC has some odd issue with the first line being loaded into the compiler (#version <openGL versions> HAS TO BE FIRST!) the #include macro from the pre-compiler just complicates code compiling, so I manually copy/pasted the #include for each file calling for them, who has a main function (thus establishing what is actually a shader, and whats a header/library).

Files in root: The shader files containing a main class, thus being the actual shader. These files come in a variety of classifications (comp, vert, frag, geom, tess, tese, etc). I gave them there classifications based on the names of the file, and how they were being used. So reclassifying them may be needed to ensure a proper SPIR-V shader is created (You should be able to infer from the code/name of the files like I did).

Compiling the code should be done calling glslc from the command line.

Once the compiler no longer complains about the code being unfit for the Vulkan spec, we can start using it to link and compile a binary for use.

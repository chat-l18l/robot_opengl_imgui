/**
 * @file mesh_import.cpp
 * @brief assimp to rbt_mesh_t.
 */

#include "mesh_import.h"

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

/** How far into a COLLADA file its <up_axis> is looked for; it sits in the <asset> header. */
#define RBT_COLLADA_HEADER_SIZE 16384

/**
 * @brief The rotation that takes a file's declared up axis onto +Z.
 *
 * Only COLLADA declares one. Y_UP turns +90 degrees about X, (x, y, z) to
 * (x, -z, y); X_UP turns -90 degrees about Y, (x, y, z) to (-z, y, x). Anything
 * else, including no declaration, is already Z-up.
 */
static Matrix3f s_up_axis_to_z(const char *path)
{
    const size_t length = strlen(path);
    if (length < 4 || strcasecmp(path + length - 4, ".dae") != 0) {
        return Matrix3f::Identity();
    }

    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return Matrix3f::Identity();
    }
    char   header[RBT_COLLADA_HEADER_SIZE + 1];
    size_t got = fread(header, 1, RBT_COLLADA_HEADER_SIZE, file);
    fclose(file);
    header[got] = '\0';

    const char *tag = strstr(header, "<up_axis>");
    if (tag == NULL) {
        return Matrix3f::Identity();
    }
    tag += strlen("<up_axis>");
    while (*tag == ' ' || *tag == '\t' || *tag == '\n' || *tag == '\r') {
        tag++;
    }

    Matrix3f rotation = Matrix3f::Identity();
    if (strncmp(tag, "Y_UP", 4) == 0) {
        rotation << 1.0f, 0.0f, 0.0f,
                    0.0f, 0.0f, -1.0f,
                    0.0f, 1.0f, 0.0f;
    } else if (strncmp(tag, "X_UP", 4) == 0) {
        rotation << 0.0f, 0.0f, -1.0f,
                    0.0f, 1.0f, 0.0f,
                    1.0f, 0.0f, 0.0f;
    }
    return rotation;
}

size_t rbt_mesh_import(const char *path, bool honor_up_axis,
                       std::vector<rbt_mesh_t> *meshes, std::vector<rbt_mesh_color_t> *colors)
{
    assert(path != NULL);
    assert(meshes != NULL);
    assert(colors != NULL);

    Assimp::Importer importer;

    /* assimp would bring every file to Y-up. That is switched off, and when
     * the declared axis is to be honoured it is applied here instead, towards
     * Z-up, so there is at most one conversion and it does not depend on what
     * assimp does to its root node. The unit scale stays on. */
    importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);

    /* Drop points and lines outright rather than importing them only to
     * skip them. */
    importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

    const unsigned int flags = aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_PreTransformVertices  /* bake the file's node tree */
                             | aiProcess_GenSmoothNormals      /* only where a mesh has none */
                             | aiProcess_SortByPType;

    const aiScene *scene = importer.ReadFile(path, flags);
    if (scene == NULL || scene->mRootNode == NULL || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)) {
        fprintf(stderr, "mesh: %s: %s\n", path, importer.GetErrorString());
        return 0;
    }

    const Matrix3f to_z_up = honor_up_axis ? s_up_axis_to_z(path) : Matrix3f::Identity();

    size_t appended = 0;
    for (unsigned int m = 0; m < scene->mNumMeshes; m++) {
        const aiMesh *source = scene->mMeshes[m];
        if (!(source->mPrimitiveTypes & aiPrimitiveType_TRIANGLE) || source->mNumVertices == 0) {
            continue;
        }

        rbt_mesh_t mesh;
        mesh.vertices.resize(source->mNumVertices);
        for (unsigned int v = 0; v < source->mNumVertices; v++) {
            const aiVector3D &p = source->mVertices[v];
            mesh.vertices[v].position = to_z_up * Vector3f(p.x, p.y, p.z);
            mesh.vertices[v].normal   = to_z_up * (source->HasNormals()
                                      ? Vector3f(source->mNormals[v].x, source->mNormals[v].y, source->mNormals[v].z)
                                      : Vector3f::UnitY());
        }

        mesh.indices.reserve((size_t)source->mNumFaces * 3);
        for (unsigned int f = 0; f < source->mNumFaces; f++) {
            const aiFace &face = source->mFaces[f];
            if (face.mNumIndices != 3) {
                continue;  /* triangulated already; anything else is degenerate */
            }
            mesh.indices.push_back(face.mIndices[0]);
            mesh.indices.push_back(face.mIndices[1]);
            mesh.indices.push_back(face.mIndices[2]);
        }
        if (mesh.indices.empty()) {
            continue;
        }

        /* assimp invents a grey "DefaultMaterial" for files that have none;
         * that is not a colour the file asked for. */
        rbt_mesh_color_t color = {{0.0f, 0.0f, 0.0f}, false};
        const aiMaterial *material = scene->mMaterials[source->mMaterialIndex];
        aiString          name;
        const bool invented = material->Get(AI_MATKEY_NAME, name) == AI_SUCCESS
                           && strcmp(name.C_Str(), AI_DEFAULT_MATERIAL_NAME) == 0;
        aiColor4D diffuse;
        if (!invented && aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS) {
            color.rgb[0]  = diffuse.r;
            color.rgb[1]  = diffuse.g;
            color.rgb[2]  = diffuse.b;
            color.present = true;
        }

        meshes->push_back(std::move(mesh));
        colors->push_back(color);
        appended++;
    }

    if (appended == 0) {
        fprintf(stderr, "mesh: %s: no triangles in the file\n", path);
    }
    return appended;
}

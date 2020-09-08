#include <math.h>
#include <fstream>
#include <sys/stat.h>
#include <sstream>
#ifdef _WIN32
	#include <direct.h>
#endif
#include <SDL.h>
#if defined(_WIN32) || defined(__linux__)
	#include <glad/glad.h>
	#include "gl2/src/glad.c"
#else
	#define GL_GLEXT_PROTOTYPES
#endif
#include <SDL_opengl.h>

#define HTTP_IMPLEMENTATION
#include "http.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_DS_IMPLEMENTATION
#include "stb_ds.h"

#include "proto/rocktree.pb.h"
#include "proto/rocktree.pb.cc"
using namespace geo_globetrotter_proto_rocktree;

#include "imgui.h"
#include "examples/imgui_impl_sdl.h"
#include "examples/imgui_impl_opengl2.h"
#include "imgui.cpp"
#include "imgui_widgets.cpp"
#include "imgui_draw.cpp"
#include "examples/imgui_impl_sdl.cpp"
#include "examples/imgui_impl_opengl2.cpp"

#include "proto_url.h"
#include "vector_math.h"
#include "util.cpp"

SDL_Window* sdl_window;

const int MAX_LEVEL = 20;

void getNodePathAndFlags(int path_id, char path[], int* level, int* flags) {
	*level = 1 + (path_id & 3);
	path_id >>= 2;
	for (int i = 0; i < *level; i++) {
		path[i] = '0' + (path_id & 7);
		path_id >>= 3;
	}
	*flags = path_id;
}

PlanetoidMetadata* getPlanetoidMetadata() {
	unsigned char* data; size_t len;
	if (!fetchData("PlanetoidMetadata", &data, &len)) return NULL;

	PlanetoidMetadata* planetoid = new PlanetoidMetadata();
	if (planetoid->ParseFromArray(data, len)) {
		free(data);
		return planetoid;
	}
	free(data);
	delete planetoid;
	return NULL;
}

BulkMetadata* getBulkMetadata(const char* path, int epoch) {
	char path_buf[200];
	sprintf(path_buf, "cache/bulk_%s_%d.bin", path, epoch);
	unsigned char* data = NULL; size_t len;
	if (!readFile(path_buf, &data, &len)) {
		char url_buf[200];
		sprintf(url_buf, "BulkMetadata/pb=!1m2!1s%s!2u%d", path, epoch);
		if (!fetchData(url_buf, &data, &len)) return NULL;
		writeFile(path_buf, data, len);
	}

	BulkMetadata* bulk = new BulkMetadata();
	if (bulk->ParseFromArray(data, len)) {
		free(data);
		return bulk;
	}
	free(data);
	delete bulk;
	return NULL;
}

NodeData* getNodeData(const char* path, int epoch, int texture_format, int imagery_epoch) {
	char path_buf[200];
	sprintf(path_buf, "cache/node_%s_%d_%d_%d.bin",
		path, epoch, texture_format, imagery_epoch);
	unsigned char* data; size_t len;
	if (!readFile(path_buf, &data, &len)) {
		char url_buf[200];
		if (imagery_epoch == -1) { // none
			sprintf(url_buf, "NodeData/pb=!1m2!1s%s!2u%d!2e%d!4b0",
				path, epoch, texture_format);
		} else {
			sprintf(url_buf, "NodeData/pb=!1m2!1s%s!2u%d!2e%d!3u%d!4b0",
				path, epoch, texture_format, imagery_epoch);
		}
		if (!fetchData(url_buf, &data, &len)) return NULL;
		writeFile(path_buf, data, len);
	}

	NodeData* node = new NodeData();
	if (node->ParseFromArray(data, len)) {
		free(data);
		return node;
	}
	free(data);
	delete node;
	return NULL;
}

struct OrientedBoundingBox {
	vec3_t center;
	vec3_t extents;
	mat3_t orientation;
};

int unpackInt(std::string packed, int* index) {
	int c = 0, d = 1;
	int e;
	do {
		e = (unsigned char)packed[(*index)++];
		c += (e & 0x7F) * d;
		d <<= 7;
	} while (e & 0x80);
	return c;
}

short unpackShort(std::string packed, int* index) {
	unsigned c = (unsigned char)packed[(*index)++];
	c |= (unsigned char)packed[(*index)++] << 8;
	//return c & 32768 ? c | 4294901760 : c; // to signed short
	return (short)c;
}

unsigned short unpackUnsignedShort(std::string packed, int* index) {
	unsigned c = (unsigned)packed[(*index)++];
	c |= (unsigned)packed[(*index)++] << 8;
	return (unsigned short)c;
}

// from decode-resource.js:407
void unpackObb(std::string data, vec3_t head_node_center, float meters_per_texel, OrientedBoundingBox* obb) {
	int i = 0;
	obb->center[0] = unpackShort(data, &i) * meters_per_texel + head_node_center[0];
	obb->center[1] = unpackShort(data, &i) * meters_per_texel + head_node_center[1];
	obb->center[2] = unpackShort(data, &i) * meters_per_texel + head_node_center[2];
	obb->extents[0] = (unsigned)data[i++] * meters_per_texel;
	obb->extents[1] = (unsigned)data[i++] * meters_per_texel;
	obb->extents[2] = (unsigned)data[i++] * meters_per_texel;
	vec3_t euler;
	euler[0] = unpackUnsignedShort(data, &i) * M_PI / 32768.0f;
	euler[1] = unpackUnsignedShort(data, &i) * M_PI / 65536.0f;
	euler[2] = unpackUnsignedShort(data, &i) * M_PI / 32768.0f;
	float c0 = cosf(euler[0]);
	float s0 = sinf(euler[0]);
	float c1 = cosf(euler[1]);
	float s1 = sinf(euler[1]);
	float c2 = cosf(euler[2]);
	float s2 = sinf(euler[2]);
	obb->orientation[0] = c0 * c2 - c1 * s0 * s2;
	obb->orientation[1] = c1 * c0 * s2 + c2 * s0;
	obb->orientation[2] = s2 * s1;
	obb->orientation[3] = -c0 * s2 - c2 * c1 * s0;
	obb->orientation[4] = c0 * c1 * c2 - s0 * s2;
	obb->orientation[5] = c2 * s1;
	obb->orientation[6] = s1 * s0;
	obb->orientation[7] = -c0 * s1;
	obb->orientation[8] = c1;
}

// from minified js (only positions)
int unpackVertices(std::string packed, unsigned char** vertices) {
	int i = 0;
	int h = packed.size();
	int k = h / 3;
	*vertices = new unsigned char[8 * k];
	for (int m = 0; m < 3; m++) { // m == stride
		int p = (unsigned char)packed[i++];
		(*vertices)[m] = p;
		for (int v = 1; v < k; v++) {
			p = (p + packed[i++]) & 0xFF;
			(*vertices)[8 * v + m] = p;
		}
	}
	return 8 * k;
}

// from minified js
void unpackTexCoords(std::string packed, unsigned char* vertices, int vertices_len) {
	int h = vertices_len / 8;
	int i = 0;
	int k = (unsigned char)packed[i++];
	k += (unsigned char)packed[i++] << 8;
	int g = (unsigned char)packed[i++];
	g += (unsigned char)packed[i++] << 8;
	assert(k == (1 << 16) - 1);
	assert(g == (1 << 16) - 1);
	int m = 0, p = 0;
	for (int B = 0; B < h; B++) {
		m = (m + ((unsigned char)packed[i + 0 * h + B] + 
				 ((unsigned char)packed[i + 2 * h + B] << 8))) & k;
		p = (p + ((unsigned char)packed[i + 1 * h + B] + 
				 ((unsigned char)packed[i + 3 * h + B] << 8))) & g;
		int A = 8 * B + 4;
		vertices[A + 0] = m & 0xFF;
		vertices[A + 1] = m >> 8;
		vertices[A + 2] = p & 0xFF;
		vertices[A + 3] = p >> 8;
	}
}

// from minified js
int unpackIndices(std::string packed, unsigned short** indices) {
	int i = 0;
	int e = unpackInt(packed, &i);
	int g = 0; // numNonDegenerateTriangles
	unsigned short* buffer = new unsigned short[e];
	for (int k = 0, m, p = 0, v = 0, z = 0; z < e; z++) {
		int B = unpackInt(packed, &i);
		m = p;
		p = v;
		v = k - B;
		buffer[z] = v;
		m != p && p != v && m != v && g++;
		B || k++;
	}

	// TODO: move to single loop
	int indices_len = 3 * g;
	*indices = new unsigned short[indices_len];
	indices_len = 0;
	for (int i = 0; i < e - 2; i++) {
		int a = buffer[i + 0];
		int b = buffer[i + 1];
		int c = buffer[i + 2];
		if (a == b || a == c || b == c) continue;
		if (i & 1) { // reverse winding
			(*indices)[indices_len++] = a;
			(*indices)[indices_len++] = c;
			(*indices)[indices_len++] = b;
		} else {
			(*indices)[indices_len++] = a;
			(*indices)[indices_len++] = b;
			(*indices)[indices_len++] = c;
		}
	}

	delete [] buffer;

	return indices_len;
}

int gpu_mem_usage = 0;

struct PlanetMesh {
	mat4_t transform;
	GLuint vertices_buffer;
	GLuint indices_buffer;
	int element_count;
	GLuint texture;
	float uv_offset[2];
	float uv_scale[2];

	void loadFromMesh(Mesh* mesh);
	void free() {
		glDeleteBuffers(1, &vertices_buffer);
		glDeleteBuffers(1, &indices_buffer);
		glDeleteTextures(1, &texture);
		vertices_buffer = indices_buffer = texture = 0;
	}
};

void PlanetMesh::loadFromMesh(Mesh* mesh) {
	unsigned short* indices;
	unsigned char* vertices;
	int indices_len = unpackIndices(mesh->indices(), &indices);
	int vertices_len = unpackVertices(mesh->vertices(), &vertices);
	unpackTexCoords(mesh->texture_coordinates(), vertices, vertices_len);
	
	uv_offset[0] = mesh->uv_offset_and_scale(0);
	uv_offset[1] = mesh->uv_offset_and_scale(1);
	uv_scale[0] = mesh->uv_offset_and_scale(2);
	uv_scale[1] = mesh->uv_offset_and_scale(3);

	glGenBuffers(1, &vertices_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertices_buffer);
	glBufferData(GL_ARRAY_BUFFER, vertices_len * sizeof(unsigned char), vertices, GL_STATIC_DRAW);
	glGenBuffers(1, &indices_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_buffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_len * sizeof(unsigned short), indices, GL_STATIC_DRAW);
	element_count = indices_len;

	delete [] indices;
	delete [] vertices;

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	assert(mesh->texture().size() == 1);
	auto tex = mesh->texture()[0];
	if (tex.format() == Texture_Format_JPG) {
		assert(tex.data().size() == 1); // else we have to concatenate
		std::string data = tex.data()[0];
		int width, height, comp;
		unsigned char* pixels = stbi_load_from_memory((unsigned char*)&data[0],
			data.size(), &width, &height, &comp, 0);
		if (pixels != NULL) {
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);
			unsigned char* pixels2 = new unsigned char[width * height];
			unsigned char* pixels_in = pixels;
			unsigned char* pixels_out = pixels2;
			for (int mipmap_level = 1; width > 1 && height > 1; mipmap_level++) {
				if (width > 1 && height > 1) {
					imageHalve(pixels_in, width, height, comp, pixels_out); width /= 2; height /= 2;
				} else if (width > 1) {
					imageHalveHorizontally(pixels_in, width, height, comp, pixels_out); width /= 2;
				} else if (height > 1) {
					imageHalveVertically(pixels_in, width, height, comp, pixels_out); height /= 2;
				}
				glTexImage2D(GL_TEXTURE_2D, mipmap_level, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels_out);
				unsigned char* tmp = pixels_in; pixels_in = pixels_out; pixels_out = tmp; // swap
			}
			delete [] pixels2;
			stbi_image_free(pixels);
		}
	} else assert(false); // TODO: other formats
}

struct PlanetNode {
	PlanetMesh mesh;

	OrientedBoundingBox obb;
	double lla_min[3]; // longitude, latitude, altitude
	double lla_max[3];
	int level;
	char path[MAX_LEVEL + 1];
	int epoch;
};

struct NodeKVP {
	char* key;
	PlanetNode* value;
};

NodeKVP* nodes = NULL;

struct PlanetBulk {
	PlanetNode nodes[120]; // 340?
	int num_nodes;
} planet_bulk;

float planet_radius;

const int supported_texture_formats[] = { // ordered by preference
	Texture_Format_CRN_DXT1,
	Texture_Format_DXT1,
	Texture_Format_JPG,
};

void loadPlanetBulk(char bulk_path[], int bulk_epoch) {
	BulkMetadata* bulk_meta = getBulkMetadata(bulk_path, bulk_epoch);
	int bulk_level = strlen(bulk_path);

	vec3_t head_node_center;
	for (int i = 0; i < 3; i++) head_node_center[i] = bulk_meta->head_node_center(i);

	//printf("%d metas\n", bulk_meta->node_metadata_size());
	for (auto node_meta : bulk_meta->node_metadata()) {
		char path[4 + 1];
		int level, flags;
		getNodePathAndFlags(node_meta.path_and_flags(), path, &level, &flags);
		path[level] = '\0';

		if (flags & NodeMetadata_Flags_NODATA) continue; // skip

		PlanetNode* node = new PlanetNode();
		shput(nodes, path, node);
		strcpy(node->path, bulk_path);
		strcat(node->path, path);
		node->level = bulk_level + level;
		node->epoch = node_meta.has_epoch() ? node_meta.epoch() : bulk_epoch;

		float meters_per_texel = node_meta.has_meters_per_texel() ?
			node_meta.meters_per_texel() : bulk_meta->meters_per_texel(level - 1);

		unpackObb(node_meta.oriented_bounding_box(), head_node_center,
			meters_per_texel, &node->obb);

		int available_texture_formats = node_meta.has_available_texture_formats() ?
			node_meta.available_texture_formats() :
			bulk_meta->default_available_texture_formats();

		int texture_format = Texture_Format_JPG;
		for (int supported_texture_format : supported_texture_formats) {
			if (available_texture_formats & (1 << (supported_texture_format - 1))) {
				texture_format = supported_texture_format;
				break;
			}
		}

		int imagery_epoch = -1; // do not use imagery epoch
		if (flags & NodeMetadata_Flags_USE_IMAGERY_EPOCH) {
			imagery_epoch = node_meta.has_imagery_epoch() ?
				node_meta.imagery_epoch() :
				bulk_meta->default_imagery_epoch();
		}

		NodeData* node_data = getNodeData(node->path, node->epoch, texture_format, imagery_epoch);
		if (node_data) {
			for (int i = 0; i < 3; i++) {
				node->lla_min[i] = node_data->kml_bounding_box(i + 0);
				node->lla_max[i] = node_data->kml_bounding_box(i + 3);
			}

			for (int i = 0; i < 16; i++) node->mesh.transform[i] = (float)node_data->matrix_globe_from_mesh(i);

			for (auto mesh : node_data->meshes()) node->mesh.loadFromMesh(&mesh);

			delete node_data;
		} else { // node == NULL
			assert(false);
		}
	}

	delete bulk_meta;
}

// TODO: use generic loadPlanetBulk
void loadPlanet() {
	PlanetoidMetadata* planetoid = getPlanetoidMetadata();
	assert(planetoid);
	printf("earth radius: %f\n", planetoid->radius());
	planet_radius = planetoid->radius();

	int root_epoch = planetoid->root_node_metadata().epoch();
	BulkMetadata* root_bulk = getBulkMetadata("", root_epoch);
	assert(root_bulk);

	vec3_t head_node_center;
	for (int i = 0; i < 3; i++) head_node_center[i] = root_bulk->head_node_center(i);

	printf("%d metas\n", root_bulk->node_metadata().size());
	planet_bulk.num_nodes = 0;
	for (auto node_meta : root_bulk->node_metadata()) {
		char path[MAX_LEVEL + 1];
		int level, flags;
		getNodePathAndFlags(node_meta.path_and_flags(), path, &level, &flags);
		path[level] = '\0';

		if (flags & NodeMetadata_Flags_NODATA) continue; // skip

		PlanetNode* node = &planet_bulk.nodes[planet_bulk.num_nodes++];
		strcpy(node->path, path);
		node->level = level;
		node->epoch = node_meta.has_epoch() ? node_meta.epoch() : root_epoch;

		float meters_per_texel = node_meta.has_meters_per_texel() ?
			node_meta.meters_per_texel() : root_bulk->meters_per_texel(level - 1);

		unpackObb(node_meta.oriented_bounding_box(), head_node_center,
			meters_per_texel, &node->obb);

		int available_texture_formats = node_meta.has_available_texture_formats() ?
			node_meta.available_texture_formats() :
			root_bulk->default_available_texture_formats();

		int texture_format = Texture_Format_JPG;
		for (int supported_texture_format : supported_texture_formats) {
			if (available_texture_formats & (1 << (supported_texture_format - 1))) {
				texture_format = supported_texture_format;
				break;
			}
		}

		int imagery_epoch = -1; // do not use imagery epoch
		if (flags & NodeMetadata_Flags_USE_IMAGERY_EPOCH) {
			imagery_epoch = node_meta.has_imagery_epoch() ?
				node_meta.imagery_epoch() :
				root_bulk->default_imagery_epoch();
		}

		NodeData* node_data = getNodeData(path, node->epoch, texture_format, imagery_epoch);
		if (node_data) {
			for (int i = 0; i < 3; i++) {
				node->lla_min[i] = node_data->kml_bounding_box(i + 0);
				node->lla_max[i] = node_data->kml_bounding_box(i + 3);
			}

			printf("%s n %.2f s %.2f w %.2f e %.2f\n", node->path,
				node->lla_max[1], node->lla_min[1],
				node->lla_min[0], node->lla_max[0]);

			for (int i = 0; i < 16; i++) node->mesh.transform[i] = (float)node_data->matrix_globe_from_mesh(i);

			for (auto mesh : node_data->meshes()) node->mesh.loadFromMesh(&mesh);

			delete node_data;
		} else { // node == NULL
			assert(false);
		}
	}

	delete root_bulk;
}

GLuint makeShader(const char* vert_src, const char* frag_src) {
	GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert_shader, 1, &vert_src, NULL);
	glCompileShader(vert_shader);
	GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag_shader, 1, &frag_src, NULL);
	glCompileShader(frag_shader);
	GLuint program = glCreateProgram();
	glAttachShader(program, vert_shader);
	glAttachShader(program, frag_shader);
	glLinkProgram(program);
	glDetachShader(program, vert_shader);
	glDetachShader(program, frag_shader);
	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
	return program;
}

GLuint program;
GLint transform_loc;
GLint uv_offset_loc;
GLint uv_scale_loc;
GLint texture_loc;
GLint position_loc;
GLint texcoords_loc;
void initGL() {
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	program = makeShader(
		"uniform mat4 transform;"
		"uniform vec2 uv_offset;"
		"uniform vec2 uv_scale;"
		"attribute vec3 position;"
		"attribute vec2 texcoords;"
		"varying vec2 v_texcoords;"
		"void main() {"
		"	v_texcoords = (texcoords + uv_offset) * uv_scale;"
		"	gl_Position = transform * vec4(position, 1.0);"
		"}",

		"#ifdef GL_ES\n"
		"precision mediump float;\n"
		"#endif\n"
		"uniform sampler2D texture;"
		"varying vec2 v_texcoords;"
		"void main() {"
		"	gl_FragColor = vec4(texture2D(texture, v_texcoords).rgb, 1.0);"
		"}"
	);
	glUseProgram(program);
	transform_loc = glGetUniformLocation(program, "transform");
	uv_offset_loc = glGetUniformLocation(program, "uv_offset");
	uv_scale_loc = glGetUniformLocation(program, "uv_scale");
	texture_loc = glGetUniformLocation(program, "texture");
	position_loc = glGetAttribLocation(program, "position");
	texcoords_loc = glGetAttribLocation(program, "texcoords");
}

// https://fgiesen.wordpress.com/2012/08/31/frustum-planes-from-the-projection-matrix/
void getFrustumPlanes(mat4_t frustum, vec4_t planes[6]) {
	mat4_t m;
	MatrixCopy(frustum, m); // we need to access the row vectors
	MatrixTranspose(m);
	for (int i = 0; i < 3; i++) {
		VectorAdd4(&m[12], &m[4 * i], planes[i + 0]);
		VectorSubtract4(&m[12], &m[4 * i], planes[i + 3]);
	}
}

// -1 inside, 0 intersect, 1 outside
int classifyObbFrustum(OrientedBoundingBox* obb, vec4_t planes[6]) {
	int result = -1; // inside
	for (int i = 0; i < 6; i++) {
		// Real-Time Collision Detection 5.2.3 Testing Box Against Plane
		vec3_t abs_plane {
			fabsf(DotProduct(planes[i], &obb->orientation[0])),
			fabsf(DotProduct(planes[i], &obb->orientation[3])),
			fabsf(DotProduct(planes[i], &obb->orientation[6])),
		};
		float r = DotProduct(obb->extents, abs_plane);
		float d = DotProduct(obb->center, planes[i]) + planes[i][3];
		if (fabsf(d) < r) result = 0; // intersect
		if (d + r < 0.0f) return 1; // outside
	}

	return result;
}

mat4_t cam_rot = {
	0.0f, 0.0f, 1.0f, 0.0f,
	1.0f, 0.0f, 0.0f, 0.0f,
	0.0f, 1.0f, 0.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 1.0f,
};
float cam_lat = 0.0f;
float cam_lon = 0.0f;
float cam_height = 0.0f;
vec3_t cam_pos;
float cam_zoom = 3.0f;
int mouse_x = 0, mouse_y = 0;
int prev_mouse_x, prev_mouse_y;

void cameraGui() {
	mat4_t inv_cam_rot;
	MatrixCopy(cam_rot, inv_cam_rot);
	MatrixTranspose(inv_cam_rot);

	vec3_t out;
	vec3_t pos = { 0.0f, 0.0f, 1.0f };
	MatrixMultiplyPosition(inv_cam_rot, pos, out);
	cam_lat = asinf(out[2]);
	cam_lon = atan2f(out[1], out[0]);
	pos[0] = pos[1] = 0;
	pos[2] = cam_height;
	MatrixMultiplyPosition(inv_cam_rot, pos, out);
	VectorCopy(out, cam_pos);

	ImGui::Begin("Camera");

	float in_cam_height = (cam_height - planet_radius) / 1000;
	ImGui::SliderAngle("lat", &cam_lat, -90, 90, "%.6f°");
	ImGui::SliderAngle("lon", &cam_lon, -180, 180, "%.6f°");
	ImGui::InputFloat("height (km)", &in_cam_height);
	cam_height = 1000 * in_cam_height + planet_radius;

	vec3_t in_cam_pos;
	VectorScale(cam_pos, 1.0f / 1000.0f, in_cam_pos);
	ImGui::InputFloat("x (km)", &in_cam_pos[0]);
	ImGui::InputFloat("y (km)", &in_cam_pos[1]);
	ImGui::InputFloat("z (km)", &in_cam_pos[2]);
	VectorScale(in_cam_pos, 1000.0f, cam_pos);

	ImGui::End();
}

void drawCube();
void drawPlanet() {
	mat4_t temp0, temp1, temp2,
		translation, rotation, scale,
		model, view, projection, modelview, viewprojection, transform;

	int width, height;
	SDL_GL_GetDrawableSize(sdl_window, &width, &height);
	glViewport(0, 0, width, height);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	cam_height = planet_radius * (1.0f + powf(1.337f, 0.0f - cam_zoom));

	float aspect_ratio = (float)width / (float)height;
	float fov = 0.25f * (float)M_PI; // 45°
	float right_plane, top_plane, near_plane = 1000.0f, far_plane = cam_height;
	// TODO: choose near plane based on altitude
	if (aspect_ratio > 1.0f) {
		right_plane = tanf(0.5f * fov) * near_plane;
		top_plane = right_plane / aspect_ratio;
	} else {
		top_plane = tanf(0.5f * fov) * near_plane;
		right_plane = aspect_ratio * top_plane;
	}
	MatrixFrustum(-right_plane, right_plane, -top_plane, top_plane,
		near_plane, far_plane, projection);

	vec3_t t = { 0.0f, 0.0f, -cam_height };

	prev_mouse_x = mouse_x; prev_mouse_y = mouse_y;
	unsigned mouse_buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
	SDL_GetWindowSize(sdl_window, &width, &height);

	vec3_t origin = { 0.0f, 0.0f, 0.0f };
	if (!ImGui::GetIO().WantCaptureMouse) {
		if (mouse_buttons & SDL_BUTTON(SDL_BUTTON_LEFT) && 
			(mouse_x != prev_mouse_x || mouse_y != prev_mouse_y)) {
			vec3_t w0, w1;
			w0[0] = (2.0f * prev_mouse_x / width - 1.0f) * right_plane / near_plane;
			w0[1] = (1.0f - 2.0f * prev_mouse_y / height) * top_plane / near_plane;
			w0[2] = -1.0f;
			VectorNormalize(w0);
			w1[0] = (2.0f * mouse_x / width - 1.0f) * right_plane / near_plane;
			w1[1] = (1.0f - 2.0f * mouse_y / height) * top_plane / near_plane;
			w1[2] = -1.0f;
			VectorNormalize(w1);
			float t0, t1;
			if (intersectRaySphere(origin, w0, t, planet_radius, &t0) && 
				intersectRaySphere(origin, w1, t, planet_radius, &t1)) {
				VectorScale(w0, t0, w0);
				VectorSubtract(w0, t, w0);
				VectorNormalize(w0);
				VectorScale(w1, t1, w1);
				VectorSubtract(w1, t, w1);
				VectorNormalize(w1);

				vec3_t axis;
				CrossProduct(w0, w1, axis);
				VectorNormalize(axis);
				float dot = DotProduct(w0, w1);
				if (dot > 1.0f) dot = 1.0f;
				float angle = acosf(dot);
				MatrixRotation(axis, angle, temp0);
				MatrixCopy(cam_rot, temp1);
				MatrixMultiply(temp0, temp1, cam_rot);
			}
		}
	}
	mat4_t inv_cam_rot;
	MatrixCopy(cam_rot, inv_cam_rot);
	MatrixTranspose(inv_cam_rot);

	vec3_t dirs[4], out;
	float min_lat = 90.0f, max_lat = -90.0f;
	float min_lon = 180.0f, max_lon = -180.0f;
	for (int i = 0; i < 4; i++) {
		out[0] = (i & 1 ? 1 : -1) * right_plane / near_plane;
		out[1] = (i & 2 ? 1 : -1) * top_plane / near_plane;
		out[2] = -1.0f;
		VectorNormalize(out);
		float t0;
		if (intersectRaySphere(origin, out, t, planet_radius, &t0)) {
			VectorScale(out, t0, out);
			VectorSubtract(out, t, out);
			VectorNormalize(out);
			MatrixMultiplyPosition(inv_cam_rot, out, dirs[i]);

			float lat = asinf(dirs[i][2]) * 180.0f / M_PI;
			float lon = atan2f(dirs[i][1], dirs[i][0]) * 180.0f / M_PI;
			min_lat = std::min(min_lat, lat);
			max_lat = std::max(max_lat, lat);
			min_lon = std::min(min_lon, lon);
			max_lon = std::max(max_lon, lon);
			ImGui::Text("lat/lon %.2f° %.2f°", lat, lon);
		}
	}

	cameraGui();

	MatrixCopy(cam_rot, rotation);
	MatrixTranslation(t, translation);
	MatrixMultiply(translation, rotation, view);
	MatrixMultiply(projection, view, viewprojection);
	vec4_t frustum_planes[6]; // for obb culling
	getFrustumPlanes(viewprojection, frustum_planes);

	int num_meshes = 0, num_meshes_culled = 0;
	int cam_level = (int)cam_zoom + 2;

	PlanetNode** draw_nodes = NULL;
	arrsetlen(draw_nodes, 0); // reset

	int offsets[] = { 0, 8, 32, 120 };
	assert(cam_level >= 2);
	int offset_index = (cam_level > 4 ? 4 : cam_level) - 2;
	for (int node_index = offsets[offset_index]; node_index < offsets[offset_index + 1]; node_index++) {
		PlanetNode* planet_node = &planet_bulk.nodes[node_index];

		//assert((cam_level > 4 ? 4 : cam_level) == planet_node->level);

		num_meshes++;
		int c = classifyObbFrustum(&planet_node->obb, frustum_planes); 
		if (c == 1) {
			num_meshes_culled++;
			continue; // cull
		}
		//if (c == 0) // intersection -> need to check child nodes

		if (cam_level > planet_node->level && planet_node->level % 4 == 0) {
			//loadPlanetBulk(planet_node->path, planet_node->epoch);
			//exit(0);
		}

		arrput(draw_nodes, planet_node);
	}

	glUseProgram(program);
	glEnableVertexAttribArray(position_loc);
	glEnableVertexAttribArray(texcoords_loc);
	for (int i = 0; i < arrlen(draw_nodes); i++) {
		PlanetNode* planet_node = draw_nodes[i];
		MatrixMultiply(viewprojection, planet_node->mesh.transform, transform);

		glUniformMatrix4fv(transform_loc, 1, GL_FALSE, transform);
		glUniform2fv(uv_offset_loc, 1, planet_node->mesh.uv_offset);
		glUniform2fv(uv_scale_loc, 1, planet_node->mesh.uv_scale);

		glUniform1i(texture_loc, 0);
		glBindTexture(GL_TEXTURE_2D, planet_node->mesh.texture);

		glBindBuffer(GL_ARRAY_BUFFER, planet_node->mesh.vertices_buffer);
		glVertexAttribPointer(position_loc, 3, GL_UNSIGNED_BYTE, GL_FALSE, 8, (void*)0);
		glVertexAttribPointer(texcoords_loc, 2, GL_UNSIGNED_SHORT, GL_FALSE, 8, (void*)4);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planet_node->mesh.indices_buffer);
		glDrawElements(GL_TRIANGLES, planet_node->mesh.element_count, GL_UNSIGNED_SHORT, NULL);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(position_loc);
	glDisableVertexAttribArray(texcoords_loc);
	glUseProgram(0);

	ImGui::Text("culled %d/%d meshes", num_meshes_culled, num_meshes);

	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projection);
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(view);
	glScalef(planet_radius, planet_radius, planet_radius);
	glDisable(GL_DEPTH_TEST);
	// draw axes
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 1.0f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 1.0f);
	glEnd();
	glPointSize(10.0f);
	glBegin(GL_POINTS);
	glColor3f(1.0f, 0.0f, 1.0f);
	glVertex3f(out[0], out[1], out[2]);
	for (int i = 0; i < 4; i++) {
		glVertex3f(dirs[i][0], dirs[i][1], dirs[i][2]);
	}
	glEnd();
	glEnable(GL_DEPTH_TEST);
}

void drawCube() {
	glBegin(GL_LINES);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);

	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);

	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glEnd();
}

void init() {
#if 0
	vec3_t x_axis = {1.0f, 0.0f, 0.0f};
	vec3_t y_axis = {0.0f, 1.0f, 0.0f};
	MatrixRotation(x_axis, (float)M_PI * cam_lat / 180.0f, x_rotation);
	MatrixRotation(y_axis, (float)M_PI * cam_lon / 180.0f, y_rotation);
	MatrixMultiply(x_rotation, y_rotation, temp0);
	mat4_t base = {
		0.0f, 0.0f, 1.0f, 0.0f,
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f,
	};
	MatrixMultiply(temp0, base, rotation);
#endif
}

bool quit = false;
void mainloop() {
	SDL_Event sdl_event;
	while (SDL_PollEvent(&sdl_event)) {
		switch (sdl_event.type) {
			case SDL_QUIT:
				quit = true;
				break;
			case SDL_KEYDOWN:
				if (sdl_event.key.keysym.sym == SDLK_ESCAPE) quit = true;
				break;
			case SDL_MOUSEWHEEL:
				if (sdl_event.wheel.y > 0) {
					if (cam_zoom < 20.0f) cam_zoom += 1.0f;
				} else if (sdl_event.wheel.y < 0) {
					if (cam_zoom > 0.0f) cam_zoom -= 1.0f;
				}
				break;
#ifdef EMSCRIPTEN // Touch controls
			case SDL_FINGERDOWN: {
				float x = sdl_event.tfinger.x;
				float y = sdl_event.tfinger.y;
			} break;
#endif
		}
		ImGui_ImplSDL2_ProcessEvent(&sdl_event);
	}

	ImGui_ImplOpenGL2_NewFrame();
	ImGui_ImplSDL2_NewFrame(sdl_window);
	ImGui::NewFrame();

	drawPlanet();

	ImGuiIO& io = ImGui::GetIO();
	ImGui::Render();
	glUseProgram(0);
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

	SDL_GL_SwapWindow(sdl_window);
}

typedef struct {
	double n, s, w, e;
} llbounds_t;

bool overlap(llbounds_t& b0, llbounds_t& b1) {
	return b0.n >= b1.s && b1.n >= b0.s && b0.w <= b1.e && b1.w <= b0.e;
};

struct NodePathBounds {
	llbounds_t bounds;
	char path[MAX_LEVEL + 1];
	int level; // == strlen(path)
};

void enumerateNodes(llbounds_t& clip, int max_level) {
	// TODO:
	// * return node paths
	NodePathBounds* stack = NULL; // DFS

	NodePathBounds npb;
	npb = { { 0.0, -90.0, -180.0, -90.0 }, "02", 2 }; arrpush(stack, npb);
	npb = { { 0.0, -90.0,  -90.0,   0.0 }, "03", 2 }; arrpush(stack, npb);
	npb = { { 0.0, -90.0,    0.0,  90.0 }, "12", 2 }; arrpush(stack, npb);
	npb = { { 0.0, -90.0,   90.0, 180.0 }, "13", 2 }; arrpush(stack, npb);
	npb = { { 90.0,  0.0, -180.0, -90.0 }, "20", 2 }; arrpush(stack, npb);
	npb = { { 90.0,  0.0,  -90.0,   0.0 }, "21", 2 }; arrpush(stack, npb);
	npb = { { 90.0,  0.0,    0.0,  90.0 }, "30", 2 }; arrpush(stack, npb);
	npb = { { 90.0,  0.0,   90.0, 180.0 }, "31", 2 }; arrpush(stack, npb);

	while (arrlen(stack) > 0) {
		npb = arrpop(stack);

		if (!overlap(clip, npb.bounds)) continue;
		printf("%s n %.2f s %.2f w %.2f e %.2f\n", npb.path, npb.bounds.n, npb.bounds.s, npb.bounds.w, npb.bounds.e);

		if (npb.level >= max_level) continue; // Do not descend

		double mid_lat = (npb.bounds.n + npb.bounds.s) / 2.0;
		double mid_lon = (npb.bounds.w + npb.bounds.e) / 2.0;

		NodePathBounds child;
		child.level = npb.level + 1;
		int magic = child.level % 2 == 1 ? 4 : 0; // I don't know why...
		
		if (npb.bounds.n == 90.0) { // no vertical split
			child.bounds = { npb.bounds.n, mid_lat, npb.bounds.w, npb.bounds.e };
			sprintf(child.path, "%s%d", npb.path, 2 + magic);
			arrpush(stack, child);
		} else {
			child.bounds = { npb.bounds.n, mid_lat, npb.bounds.w, mid_lon };
			sprintf(child.path, "%s%d", npb.path, 2 + magic);
			arrpush(stack, child);
			child.bounds = { npb.bounds.n, mid_lat, mid_lon, npb.bounds.e };
			sprintf(child.path, "%s%d", npb.path, 3 + magic);
			arrpush(stack, child);
		}

		if (npb.bounds.s == -90.0) { // no vertical split
			child.bounds = { mid_lat, npb.bounds.s, npb.bounds.w, npb.bounds.e };
			sprintf(child.path, "%s%d", npb.path, 0 + magic);
			arrpush(stack, child);
		} else {
			child.bounds = { mid_lat, npb.bounds.s, npb.bounds.w, mid_lon };
			sprintf(child.path, "%s%d", npb.path, 0 + magic);
			arrpush(stack, child);
			child.bounds = { mid_lat, npb.bounds.s, mid_lon, npb.bounds.e };
			sprintf(child.path, "%s%d", npb.path, 1 + magic);
			arrpush(stack, child);
		}
	}
}

void testProtoUrlEncode() {
	// https://github.com/marin-m/pbtk/wiki/Protobuf-URL
	NodeDataRequest req;
	NodeKey* node_key = new NodeKey();
	node_key->set_path("0123456789abcdef");
	node_key->set_epoch(5);
	req.set_allocated_node_key(node_key);
	req.set_texture_format(Texture_Format_CRN_DXT1);
	req.set_imagery_epoch(5);
	//std::cout << "req: \"" << req.SerializeAsString() << "\"" << std::endl;

	auto url = "NodeData/pb=" + protoUrlEncode(req);
	std::cout << url << std::endl;

	char expect_url[200];
	sprintf(expect_url, "NodeData/pb=!1m2!1s%s!2u%d!2e%d!3u%d", req.node_key().path().c_str(), req.node_key().epoch(), req.texture_format(), req.imagery_epoch());
	printf("%s\n", expect_url);
	assert(!strcmp(url.c_str(), expect_url));
}

int main(int argc, char* argv[]) {
	llbounds_t clip = { 10.0, -10.0, -10.0, 10.0 };
	//llbounds_t clip = { 10.0, -10.0, 170.0, -170.0 }; // TODO
	enumerateNodes(clip, 1); // test
	testProtoUrlEncode();

	// create cache directory
	createDir("cache");

	int video_width = 768;
	int video_height = 768;

	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		fprintf(stderr, "Couldn't init SDL2: %s\n", SDL_GetError());
		exit(1);
	}
#ifdef EMSCRIPTEN
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
		SDL_GL_CONTEXT_PROFILE_ES);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#else
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
		SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
#endif
	//SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	//SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
	sdl_window = SDL_CreateWindow("Earth Client", 
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
		video_width, video_height, 
		SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	if (!sdl_window) {
		fprintf(stderr, "Couldn't create window: %s\n", SDL_GetError());
		exit(1);
	}
	SDL_GLContext gl_context = SDL_GL_CreateContext(sdl_window);
	if (!gl_context) {
		fprintf(stderr, "Couldn't create OpenGL context: %s\n", SDL_GetError());
		exit(1);
	}
	SDL_GL_SetSwapInterval(1);

#if defined(_WIN32) || defined(__linux__)
	// init glad
	if (!gladLoadGL()) {
		fprintf(stderr, "Failed to init glad\n");
		exit(1);
	}
#endif

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsLight();

	ImGui_ImplSDL2_InitForOpenGL(sdl_window, gl_context);
	ImGui_ImplOpenGL2_Init();

	initGL();
	loadPlanet();
	init();

#ifdef EMSCRIPTEN
	emscripten_set_main_loop(mainloop, 0, 1);
#else
	while (!quit) mainloop();
#endif

	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	SDL_GL_DeleteContext(gl_context);
	SDL_DestroyWindow(sdl_window);
	SDL_Quit();
	
	return 0;
}

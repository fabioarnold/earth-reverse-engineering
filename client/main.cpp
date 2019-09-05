#include <fstream>
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

#include "proto/rocktree.pb.h"
#include "proto/rocktree.pb.cc"
using namespace geo_globetrotter_proto_rocktree;

#include "vector_math.h"

SDL_Window* sdl_window;

const int MAX_LEVEL = 20;

typedef struct {
	double n, s, w, e;
} llbounds_t;

void latLonToOctant(double lat, double lon, char octant[MAX_LEVEL]) {
	octant[0] = 0;
	octant[1] = 0;
	llbounds_t box;

	if (lat < 0.0) { octant[1] |= 2; box.n = 0.0; box.s = -90.0;}
	else { octant[0] |= 2; box.n = 90.0; box.s = 0.0; }

	if (lon < -90.0) { box.w = -180.0; box.e = -90.0; }
	else if (lon < 0.0) { octant[1] |= 1; box.w = -90.0; box.e = 0.0; }
	else if (lon < 90.0) { octant[0] |= 1; box.w = 0.0; box.e = 90.0; }
	else { octant[0] |= 1; octant[1] |= 1; box.w = 90.0; box.e = 180.0; }

	int level = MAX_LEVEL;
	for (int i = 2; i < level; i++) {
		octant[i] = 0;

		double mid_lat = (box.n + box.s) / 2.0;
		double mid_lon = (box.w + box.e) / 2.0;

		if (lat < mid_lat) {
			box.n = mid_lat;
		} else {
			box.s = mid_lat;
			octant[i] |= 2;
		}

		if (lon < mid_lon) {
			box.e = mid_lon;
		} else {
			box.w = mid_lon;
			octant[i] |= 1;
		}
	}

	// to ascii
	for (int i = 0; i < level; i++) octant[i] += '0';
}

void getPathAndFlags(int path_id, char path[], int* level, int* flags) {
	*level = 1 + (path_id & 3);
	path_id >>= 2;
	for (int i = 0; i < *level; i++) {
		path[i] = '0' + (path_id & 7);
		path_id >>= 3;
	}
	*flags = path_id;
}

bool fetchData(const char* path, unsigned char** data, size_t* len) {
	const char* base_url = "http://kh.google.com/rt/earth/";
	char* url = (char*)malloc(strlen(base_url) + strlen(path) + 1);
	strcpy(url, base_url); strcat(url, path);
	printf("GET %s\n", url);
	http_t* request = http_get(url, NULL); 
	free(url);
	if (!request) return false;
	
	http_status_t status;
	do {
		status = http_process(request);
		SDL_Delay(1);
	} while (status == HTTP_STATUS_PENDING);

	if (status == HTTP_STATUS_FAILED) {
		http_release(request);
		return false;
	}

	*data = (unsigned char*)malloc(request->response_size);
	*len = request->response_size;
	memcpy(*data, request->response_data, *len);
	
	http_release(request);
	return true;
}

PlanetoidMetadata* getPlanetoid() {
	unsigned char* data; size_t len;
	if (fetchData("PlanetoidMetadata", &data, &len)) {
		PlanetoidMetadata* planetoid = new PlanetoidMetadata();
		if (planetoid->ParseFromArray(data, len)) {
			free(data);
			return planetoid;
		} else {
			free(data);
			delete planetoid;
		}
	}
	return NULL;
}

BulkMetadata* getBulk(const char* path, int epoch) {
	char url_buf[200];
	sprintf(url_buf, "BulkMetadata/pb=!1m2!1s%s!2u%d", path, epoch);
	unsigned char* data; size_t len;
	if (fetchData(url_buf, &data, &len)) {
		BulkMetadata* bulk = new BulkMetadata();
		if (bulk->ParseFromArray(data, len)) {
			free(data);
			return bulk;
		} else {
			free(data);
			delete bulk;
		}
	}
	return NULL;
}

NodeData* getNode(const char* path, int epoch, int texture_format, int imagery_epoch) {
	char url_buf[200];
	sprintf(url_buf, "NodeData/pb=!1m2!1s%s!2u%d!2e%d!3u%d!4b0", path, epoch, texture_format, imagery_epoch);
	unsigned char* data; size_t len;
	if (fetchData(url_buf, &data, &len)) {
		NodeData* node = new NodeData();
		if (node->ParseFromArray(data, len)) {
			free(data);
			return node;
		} else {
			free(data);
			delete node;
		}
	}
	return NULL;
}

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

// from minified js
int unpackIndices(std::string packed, unsigned short** indices) {
	int i = 0;
	int e = unpackInt(packed, &i);
	*indices = new unsigned short[e];
	for (int k = 0, g = 0, m, p = 0, v = 0, z = 0; z < e; z++) {
		int B = unpackInt(packed, &i);
		m = p;
		p = v;
		v = k - B;
		(*indices)[z] = v;
		m != p && p != v && m != v && g++;
		B || k++;
	}
	// g == numNonDegenerateTriangles

	// TODO: make buffer of nondegenerates

	return e;
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
	k += (unsigned char)packed[i++] << 8; // 65535
	int g = (unsigned char)packed[i++];
	g += (unsigned char)packed[i++] << 8; // 65535
	int m = 0, p = 0;
	for (int B = 0; B < h; B++) {
		m = (m + ((unsigned char)packed[i + 0 * h + B] + 
				 ((unsigned char)packed[i + 2 * h + B] << 8))) & k; // 18418
		p = (p + ((unsigned char)packed[i + 1 * h + B] + 
				 ((unsigned char)packed[i + 3 * h + B] << 8))) & g; // 49234
		int A = 8 * B + 4;
		vertices[A + 0] = m & 0xFF;
		vertices[A + 1] = m >> 8;
		vertices[A + 2] = p & 0xFF;
		vertices[A + 3] = p >> 8;
	}
}

struct Mesh {
	GLuint vertices_buffer;
	GLuint indices_buffer;
	int element_count;
} planet_mesh;

void loadPlanet() {
	PlanetoidMetadata* planetoid = getPlanetoid();
	printf("earth radius: %f\n", planetoid->radius());
	int root_epoch = planetoid->root_node_metadata().epoch();
	BulkMetadata* root_bulk = getBulk("", root_epoch);
	printf("%d root metas\n", root_bulk->node_metadata().size());
	//printf("head node key path: %s epoch: %d\n", root_bulk->head_node_key().path().c_str(), root_bulk->head_node_key().epoch());
	for (auto node_meta : root_bulk->node_metadata()) {
		char path[MAX_LEVEL+1];
		int level, flags;
		getPathAndFlags(node_meta.path_and_flags(), path, &level, &flags);
		path[level] = '\0';
		printf("path %.*s flag %d\n", level, path, flags);

		if (!(flags & NodeMetadata_Flags_NODATA)) {
			int imagery_epoch = node_meta.imagery_epoch();
			if (flags & NodeMetadata_Flags_USE_IMAGERY_EPOCH) {
				imagery_epoch = root_bulk->default_imagery_epoch();
			}
			NodeData* node = getNode(path, root_epoch, Texture_Format_DXT1, imagery_epoch);
			if (node) {
				for (auto mesh : node->meshes()) {
					unsigned short* indices;
					unsigned char* vertices;
					int indices_len = unpackIndices(mesh.indices(), &indices);
					int vertices_len = unpackVertices(mesh.vertices(), &vertices);
					unpackTexCoords(mesh.texture_coordinates(), vertices, vertices_len);
					
					// remove degenerates
					unsigned short* indices2 = new unsigned short[indices_len];
					int indices2_len = 0;
					for (int i = 0; i < indices_len - 2; i++) {
						int a = indices[i + 0];
						int b = indices[i + 1];
						int c = indices[i + 2];
						if (a == b || a == c || b == c) continue;
						// TODO: if (i & 1) reverse winding
						indices2[indices2_len++] = a;
						indices2[indices2_len++] = b;
						indices2[indices2_len++] = c;
					}

					glGenBuffers(1, &planet_mesh.vertices_buffer);
					glBindBuffer(GL_ARRAY_BUFFER, planet_mesh.vertices_buffer);
					glBufferData(GL_ARRAY_BUFFER, vertices_len * sizeof(unsigned char), vertices, GL_STATIC_DRAW);
					glGenBuffers(1, &planet_mesh.indices_buffer);
					glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planet_mesh.indices_buffer);
					glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices2_len * sizeof(unsigned short), indices2, GL_STATIC_DRAW);
					planet_mesh.element_count = indices_len;

					delete [] indices;
					delete [] indices2;
					delete [] vertices;

					assert(mesh.texture().size == 1);
					auto tex = mesh.texture().at(0);
					if (tex.format() == Texture_Format_JPG) {
						printf("tex format: %d\n", tex.format());
						std::ofstream outfile;
						outfile.open("tex.jpg", std::ios::out | std::ios::trunc | std::ios::binary);
						assert(tex.data().size == 1); // else we have to concatenate
						for (std::string data : tex.data()) {
							outfile << data;
						}
						outfile.close();
					}
					int width, height, comp;
					unsigned char* pixels = stbi_load("tex.jpg", &width, &height, &comp, 3);
					if (!pixels) {
						fprintf(stderr, "Could not load texture\n");
					}
				}
				delete node;

				return; // stop after first node for now
			}
		}

		// next level
		if (level == 4 && !(flags & NodeMetadata_Flags_LEAF)) { // bulk
			BulkMetadata* bulk = getBulk(path, root_epoch);
			if (bulk != NULL) {
				printf("metas %d\n", bulk->node_metadata().size());
				for (auto meta2 : bulk->node_metadata()) {
					getPathAndFlags(meta2.path_and_flags(), path, &level, &flags);
				}
				delete bulk;
			}
		}
	}
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
GLint color_loc;
void initGL() {
	program = makeShader(
		"uniform mat4 transform;"
		"attribute vec3 position;"
		"void main() {"
		"	gl_Position = transform * vec4(position, 1.0);"
		"}",

		"#ifdef GL_ES\n"
		"precision mediump float;\n"
		"#endif\n"
		"uniform vec3 color;"
		"void main() {"
		"	gl_FragColor = vec4(color, 1.0);"
		"}"
	);
	glUseProgram(program);
	transform_loc = glGetUniformLocation(program, "transform");
	color_loc = glGetUniformLocation(program, "color");
}

void drawPlanet() {
	int width, height;
	SDL_GL_GetDrawableSize(sdl_window, &width, &height);
	glViewport(0, 0, width, height);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glUseProgram(program);
	mat4_t transform;
	float s = 1.0f / 4.0f;
	MatrixScale(vec3_t {s, s, s}, transform);
	MatrixOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, transform);
	//MatrixIdentity(transform);
	static float angle = 0.0f;
	angle += 0.02f;
	MatrixRotation(vec3_t {0.0f, 1.0f, 0.0f}, angle, transform);
	glUniformMatrix4fv(transform_loc, 1, GL_FALSE, transform);

	glUniform3f(color_loc, 1.0f, 0.0f, 1.0f);

	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, planet_mesh.vertices_buffer);
	glVertexAttribPointer(0, 3, GL_UNSIGNED_BYTE, GL_TRUE, 8, NULL);
	//glDrawArrays(GL_TRIANGLES, 0, 3);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planet_mesh.indices_buffer);
	glDrawElements(GL_TRIANGLES, planet_mesh.element_count, GL_UNSIGNED_SHORT, NULL);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	SDL_GL_SwapWindow(sdl_window);
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
#ifdef EMSCRIPTEN // Touch controls
			case SDL_FINGERDOWN: {
				float x = sdl_event.tfinger.x;
				float y = sdl_event.tfinger.y;
			} break;
#endif
		}
	}

	drawPlanet();
}

int main(int argc, char* argv[]) {
	int video_width = 512;
	int video_height = 512;

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

	initGL();
	loadPlanet();

#ifdef EMSCRIPTEN
	emscripten_set_main_loop(mainloop, 0, 1);
#else
	while (!quit) mainloop();
#endif

	SDL_Quit();
	
	return 0;
}
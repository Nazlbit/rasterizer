#include <cstdio>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

namespace math
{
    union vec2
    {
        struct
        {
            float x, y;
        };
        float values[2];
    };

    inline vec2 operator-(const vec2 &a, const vec2 &b)
    {
        return {a.x - b.x, a.y - b.y};
    }

    inline vec2 operator+(const vec2 &a, const vec2 &b)
    {
        return {a.x + b.x, a.y + b.y};
    }

    inline vec2 operator*(const vec2 &a, const float &b)
    {
        return {a.x * b, a.y * b};
    }

    inline vec2 operator/(const vec2 &a, const float &b)
    {
        return {a.x / b, a.y / b};
    }

    inline float cross(const vec2 &a, const vec2 &b)
    {
        return a.x * b.y - b.x * a.y;
    }

    inline float edge_func(vec2 a, vec2 b, vec2 p)
    {
        return cross(p - a, p - b);
    }

    union vec3
    {
        struct
        {
            float x, y, z;
        };
        float values[3];
        vec2 xy; // type-punning -> undefined bahavior
    };

    inline vec3 operator+(const vec3 &a, const vec3 &b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    inline vec3 operator-(const vec3 &a, const vec3 &b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    inline vec3 operator-(const vec3 &a)
    {
        return {-a.x, -a.y, -a.z};
    }

    inline vec3 operator*(const vec3 &a, const float &b)
    {
        return {a.x * b, a.y * b, a.z * b};
    }

    inline vec3 operator/(const vec3 &a, const float &b)
    {
        return {a.x / b, a.y / b, a.z / b};
    }

    union vec4
    {
        struct
        {
            float x, y, z, w;
        };
        float values[4];
        vec3 xyz; // type-punning -> undefined bahavior
    };

    union mat4
    {
        vec4 row[4];
        float values[4][4];
    };

    const mat4 identity_matrix = {1.f, 0.f, 0.f, 0.f,
                                  0.f, 1.f, 0.f, 0.f,
                                  0.f, 0.f, 1.f, 0.f,
                                  0.f, 0.f, 0.f, 1.f};

    inline float dot(const vec3 &a, const vec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline float dot(const vec4 &a, const vec4 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    inline vec3 cross(const vec3 &a, const vec3 &b)
    {
        return {a.y * b.z - b.y * a.z,
                a.z * b.x - a.x * b.z,
                a.x * b.y - b.x * a.y};
    }

    inline vec3 normalize(const vec3 &v)
    {
        return v / sqrtf(dot(v, v));
    }

    inline vec3 reflect(const vec3 &normal, const vec3 &v)
    {
        return v - normal * (2.f * dot(normal, v));
    }

    mat4 operator*(const mat4 &a, const mat4 &b)
    {
        mat4 result;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result.values[i][j] = dot(a.row[i], {b.values[0][j],
                                                     b.values[1][j],
                                                     b.values[2][j],
                                                     b.values[3][j]});
            }
        }
        return result;
    }

    inline vec4 operator*(const mat4 &a, const vec4 &b)
    {
        return {dot(a.row[0], b), dot(a.row[1], b), dot(a.row[2], b), dot(a.row[3], b)};
    }

    inline mat4 make_perspective_projection_matrix(float fov, float near, float far, float aspect_ratio)
    {
        const float tg = 1.f / tanf(fov * 0.5f);
        return {tg / aspect_ratio, 0, 0, 0,
                0, tg, 0, 0,
                0, 0, far / (far - near), -far * near / (far - near),
                0, 0, 1, 0};
    }

    mat4 make_look_to_matrix(vec3 pos, vec3 dir, vec3 up)
    {
        dir = normalize(dir);
        vec3 right = normalize(cross(up, dir));
        up = cross(dir, right);

        return mat4{right.x, right.y, right.z, -dot(pos, right),
                    up.x, up.y, up.z, -dot(pos, up),
                    dir.x, dir.y, dir.z, -dot(pos, dir),
                    0, 0, 0, 1};
    }

    vec3 barycentric(const vec2 &a, const vec2 &b, const vec2 &c, const vec2 &p)
    {
        const float area = edge_func(a, b, c);
        const float C = edge_func(a, b, p) / area;
        const float A = edge_func(b, c, p) / area;
        const float B = edge_func(c, a, p) / area;

        return {A, B, C};
    }

    inline bool inside(const vec3 &barycentric)
    {
        return barycentric.x >= 0.f && barycentric.y >= 0.f && barycentric.z >= 0.f;
    }

    template <class T>
    inline T min(const T &a, const T &b)
    {
        return a < b ? a : b;
    }

    template <class T>
    inline T max(const T &a, const T &b)
    {
        return a > b ? a : b;
    }

    template <class T>
    inline T clamp(const T &min_val, const T &max_val, const T &val)
    {
        return min(max(val, min_val), max_val);
    }

    inline int cartesian_to_screen(const float val, const int screen_dimension)
    {
        return static_cast<int>((val + 1.f) * 0.5f * screen_dimension + 0.5f);
    }

} /* namespace math */

struct vertex_in
{
    math::vec3 pos;
    math::vec3 norm;
    float shade;
};

struct vertex_out
{
    math::vec3 world_pos;
    math::vec3 world_norm;
    float shade;
};

struct mesh
{
    std::vector<vertex_in> vertices;
    std::vector<unsigned> indices;
};

struct vertex_uniforms
{
    math::mat4 world;
    math::mat4 view;
    math::mat4 projection;
    math::mat4 world_norm;
};

struct fragment_uniforms
{
    math::vec3 cam_pos;
    math::vec3 light_dir;
    float ambient;
    float specular;
    float specular_pow;
    float light;
};

char shade_to_ascii(float val)
{
    const char gradient[] = {' ', '.', '-', ':', '+', '=', '*', '#', '%', '@'};
    return gradient[math::clamp<unsigned>(0u, 9u, static_cast<unsigned>(val * 10.f))];
}

math::vec4 vertex_shader(const vertex_in &in, vertex_out &out, const vertex_uniforms &uniforms)
{
    math::vec4 pos = {in.pos.x, in.pos.y, in.pos.z, 1.f};
    math::vec4 norm = {in.norm.x, in.norm.y, in.norm.z, 1.f};

    pos = uniforms.world * pos;
    out.world_pos = pos.xyz;
    pos = uniforms.projection * uniforms.view * pos;

    norm = uniforms.world_norm * norm;
    out.world_norm = {norm.x, norm.y, norm.z};

    out.shade = in.shade;

    return pos;
}

std::vector<math::vec3> shade_vertices(const std::vector<vertex_in> &in, std::vector<vertex_out> &out, const vertex_uniforms &uniforms)
{
    std::vector<math::vec3> cartesian_positions(in.size());

    for (size_t i = 0; i < in.size(); i++)
    {
        const math::vec4 pos = vertex_shader(in[i], out[i], uniforms);
        cartesian_positions[i] = pos.xyz / pos.w; // perspective division
    }

    return cartesian_positions;
}

float fragment_shader(const vertex_out &in, const fragment_uniforms &frag_uniforms)
{
    const math::vec3 normal = math::normalize(in.world_norm);
    const float diffuse = math::max(0.f, math::dot(normal, -frag_uniforms.light_dir)) * frag_uniforms.light;

    const math::vec3 cam_dir = math::normalize(in.world_pos - frag_uniforms.cam_pos);
    const float specular = powf(math::max(0.f, math::dot(-math::normalize(cam_dir), math::reflect(normal, frag_uniforms.light_dir))), frag_uniforms.specular_pow) * frag_uniforms.specular * frag_uniforms.light;
    float result = (diffuse + frag_uniforms.ambient) * in.shade + specular;

    return result;
}

void render(char *buffer, float *depth_buffer, const int width, const int height, const mesh &m, const vertex_uniforms &vert_uniforms, const fragment_uniforms &frag_uniforms)
{
    const math::vec2 pixel_half_size = {1.f / width, 1.f / height};

    std::vector<vertex_out> shaded_vertices(m.vertices.size());
    std::vector<math::vec3> cartesian_positions = shade_vertices(m.vertices, shaded_vertices, vert_uniforms);

    const size_t triangle_num = m.indices.size() / 3;

    for (size_t i = 0; i < triangle_num; i++)
    {
        const vertex_out &a_out = shaded_vertices[m.indices[i * 3]];
        const vertex_out &b_out = shaded_vertices[m.indices[i * 3 + 1]];
        const vertex_out &c_out = shaded_vertices[m.indices[i * 3 + 2]];

        const math::vec3 &a_pos = cartesian_positions[m.indices[i * 3]];
        const math::vec3 &b_pos = cartesian_positions[m.indices[i * 3 + 1]];
        const math::vec3 &c_pos = cartesian_positions[m.indices[i * 3 + 2]];

        if (math::edge_func(a_pos.xy, b_pos.xy, c_pos.xy) <= 0)
        {
            continue;
        }

        // Bounding box
        const int left = math::clamp<int>(0, width, math::cartesian_to_screen(math::min(math::min(a_pos.x, b_pos.x), c_pos.x), width));
        const int right = math::clamp<int>(0, width, math::cartesian_to_screen(math::max(math::max(a_pos.x, b_pos.x), c_pos.x), width));
        const int bottom = math::clamp<int>(0, height, math::cartesian_to_screen(-math::min(math::min(a_pos.y, b_pos.y), c_pos.y), height));
        const int top = math::clamp<int>(0, height, math::cartesian_to_screen(-math::max(math::max(a_pos.y, b_pos.y), c_pos.y), height));

        if (left == right)
        {
            continue;
        }

        for (int y = top; y < bottom; y++)
        {
            for (int x = left; x < right; x++)
            {
                const math::vec2 p = {((float)x / width) * 2.f - 1.f + pixel_half_size.x,
                                      -(((float)y / height) * 2.f - 1.f + pixel_half_size.y)};

                const math::vec3 barycentric = math::barycentric(a_pos.xy, b_pos.xy, c_pos.xy, p);
                if (inside(barycentric))
                {
                    /* Early depth test */
                    const float depth = dot({a_pos.z, b_pos.z, c_pos.z}, barycentric);
                    if (depth < 0.f || depth > depth_buffer[width * y + x])
                    {
                        continue;
                    }
                    depth_buffer[width * y + x] = depth;

                    vertex_out fragment_in;
                    fragment_in.world_pos = a_out.world_pos * barycentric.x + b_out.world_pos * barycentric.y + c_out.world_pos * barycentric.z;
                    fragment_in.world_norm = a_out.world_norm * barycentric.x + b_out.world_norm * barycentric.y + c_out.world_norm * barycentric.z;
                    fragment_in.shade = dot({a_out.shade, b_out.shade, c_out.shade}, barycentric);

                    buffer[(width + 1) * y + x] = shade_to_ascii(fragment_shader(fragment_in, frag_uniforms));
                }
            }
        }
    }
}

mesh make_sphere(int longitude_segments, int latitude_segments)
{
    longitude_segments = math::max(longitude_segments, 3);
    latitude_segments = math::max(latitude_segments, 2);

    const size_t num_vertices = (size_t)longitude_segments * (latitude_segments - 1) + 2;
    const size_t num_triangles = (size_t)longitude_segments * 2 * (latitude_segments - 1);
    const size_t num_indices = num_triangles * 3;

    mesh sphere_mesh;
    sphere_mesh.vertices.resize(num_vertices);
    sphere_mesh.indices.resize(num_indices);

    /* Generating vertices */

    /* Sides */
    for (size_t i = 0; i < longitude_segments; i++)
    {
        const float longitude = M_PI * 2 * i / longitude_segments;
        for (size_t j = 0; j < latitude_segments - 1; j++)
        {
            const float latitude = M_PI * (j + 1) / latitude_segments;
            const math::vec3 v = {sinf(latitude) * cosf(longitude), cosf(latitude), sinf(latitude) * sinf(longitude)};

            sphere_mesh.vertices[(latitude_segments - 1) * i + j] = {v, v, 1.f};
        }
    }

    /* Poles */
    sphere_mesh.vertices[num_vertices - 2] = {{0.f, 1.f, 0.f}, {0.f, 1.f, 0.f}, 1.f};   /* Top */
    sphere_mesh.vertices[num_vertices - 1] = {{0.f, -1.f, 0.f}, {0.f, -1.f, 0.f}, 1.f}; /* Bottom */

    /* Generating indices */

    /* Sides */
    size_t index_offset = 0;
    for (size_t i = 0; i < longitude_segments - 1; i++)
    {
        for (size_t j = 0; j < latitude_segments - 2; j++)
        {
            const size_t vertex_offset = j + (latitude_segments - 1) * i;
            sphere_mesh.indices[index_offset++] = vertex_offset;
            sphere_mesh.indices[index_offset++] = vertex_offset + 1;
            sphere_mesh.indices[index_offset++] = vertex_offset + latitude_segments - 1;
            sphere_mesh.indices[index_offset++] = vertex_offset + 1;
            sphere_mesh.indices[index_offset++] = vertex_offset + latitude_segments;
            sphere_mesh.indices[index_offset++] = vertex_offset + latitude_segments - 1;
        }
    }

    /* Closing loop segment */
    for (size_t j = 0; j < latitude_segments - 2; j++)
    {
        const size_t vertex_offset = j + (latitude_segments - 1) * (longitude_segments - 1);
        sphere_mesh.indices[index_offset++] = vertex_offset;
        sphere_mesh.indices[index_offset++] = vertex_offset + 1;
        sphere_mesh.indices[index_offset++] = j;
        sphere_mesh.indices[index_offset++] = vertex_offset + 1;
        sphere_mesh.indices[index_offset++] = j + 1;
        sphere_mesh.indices[index_offset++] = j;
    }

    /* Poles */
    for (size_t i = 0; i < longitude_segments - 1; i++)
    {
        /* Top */
        sphere_mesh.indices[index_offset++] = num_vertices - 2;
        sphere_mesh.indices[index_offset++] = i * (latitude_segments - 1);
        sphere_mesh.indices[index_offset++] = (i + 1) * (latitude_segments - 1);

        /* Bottom */
        sphere_mesh.indices[index_offset++] = num_vertices - 1;
        sphere_mesh.indices[index_offset++] = (latitude_segments - 1) * (i + 2) - 1;
        sphere_mesh.indices[index_offset++] = (latitude_segments - 1) * (i + 1) - 1;
    }

    /* Closing loop segment */
    sphere_mesh.indices[index_offset++] = num_vertices - 2;
    sphere_mesh.indices[index_offset++] = (longitude_segments - 1) * (latitude_segments - 1);
    sphere_mesh.indices[index_offset++] = 0;

    sphere_mesh.indices[index_offset++] = num_vertices - 1;
    sphere_mesh.indices[index_offset++] = latitude_segments - 2;
    sphere_mesh.indices[index_offset++] = (latitude_segments - 1) * longitude_segments - 1;

    return sphere_mesh;
}

void clear_framebuffer(char *buffer, float *depth_buffer, int width, int height)
{
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            buffer[(width + 1) * i + j] = ' ';
            depth_buffer[width * i + j] = 1.f;
        }
        buffer[(width + 1) * i + width] = '\n';
    }
}

int main(int, char **)
{
    const int width = 100, height = 50;
    const float aspect_ratio = (float)width / height * 0.5f;
    const float fov = M_PI / 180.f * 65.f;
    float t = 0;

    mesh m = make_sphere(50, 25);

    char screen[(width + 1) * height];
    float depth_buffer[width * height];

    while (true)
    {
        clear_framebuffer(screen, depth_buffer, width, height);

        const math::vec3 cam_dir = {-sin(t), 0.f, cos(t)};
        const math::vec3 cam_pos = cam_dir * -2.f;

        vertex_uniforms vert_uniforms;
        vert_uniforms.world = math::identity_matrix;
        vert_uniforms.world_norm = math::identity_matrix;
        vert_uniforms.view = math::make_look_to_matrix(cam_pos, cam_dir, {0.f, 1.f, 0.f});
        vert_uniforms.projection = math::make_perspective_projection_matrix(fov, 0.1f, 10.f, aspect_ratio);

        fragment_uniforms frag_uniforms;
        frag_uniforms.cam_pos = cam_pos;
        frag_uniforms.light_dir = math::normalize({-1.f, -1.f, 1.f});
        frag_uniforms.ambient = 0.15f;
        frag_uniforms.specular = 0.9f;
        frag_uniforms.specular_pow = 32.f;
        frag_uniforms.light = 0.7f;

        render(screen, depth_buffer, width, height, m, vert_uniforms, frag_uniforms);

        screen[(width + 1) * height - 1] = '\0';
        printf("%s\n", screen);

        t += 0.05;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}

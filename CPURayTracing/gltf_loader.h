#include <fstream>
#include <iostream>

#include "json.h"
#include "stb_image.h"
using json = nlohmann::json;

// ����:
// ���䴩 camera, animation, skin
// accesor �P bufferView, attribute ���������Y:
// 1. accessor �|���V bufferView�A�y�z bufferView
// ���V����Ƹs�����O(���h�֭ӡB���O�O�ƻ�)�A�L�׬O mesh, skin, animation ��
// binary ��Ƴ��|�z�L Accesor �Ө��o
// 2. bufferView �|���V buffer �����@�q��ơA�y�z��ƪ��γ~�A�H�ΥL�b
// buffer(.bin file) ������

// ���n���|�е� required�A�H�Φp�G���w�]�Ȥ]�|�Х�
// �S�����ܴN�O optional

// ��~��X����Ƶ��c
enum class DataType {
  kFloat,
  kByte,
  kUnsignedByte,
  kShort,
  kUnsignedShort,
  kInt,
  kUnsignedInt,
};
enum class PrimitiveMode {
  kPoints,
  kLines,
  kLineLoop,
  kLineStrip,
  kTriangles,
  kTriangleStrip,
  kTriangleFan,
};
// �Ψӹ�~��X�� Primitives
struct OutputPrimitives {
  // �o�|�ӬO VBO�A�H�ΦU�� VBO ����ƫ��A�A�٦� component ���ƶq
  // TODO: �p���קK�@���ƻs? �ڱq bin �ƻs�� vector�A���D�S�n�ƻs��ϥΪ̨�?
  std::vector<uint8_t> positions;
  std::vector<uint8_t> normals;
  std::vector<uint8_t> uvs;
  std::vector<uint8_t> tangents;
  DataType pos_type;
  DataType normal_type;
  DataType uv_type;
  DataType tan_type;
  int pos_comp;
  int normal_comp;
  int uv_comp;
  int tan_comp;

  // �o�O EBO
  DataType indices_type;
  std::vector<uint8_t> indices;

  // �o�O primative ���զ��覡
  PrimitiveMode mode;

  bool use_indices = false;
  // Material Information
  // Material material;
};

namespace gltf {

#define SAMPLER_MAG_FILTER_NEAREST 9728
#define SAMPLER_MAG_FILTER_LINEAR 9729

#define SAMPLER_MIN_FILTER_NEAREST 9728
#define SAMPLER_MIN_FILTER_LINEAR 9729
#define SAMPLER_MIN_FILTER_NEAREST_MIPMAP_NEAREST 9984
#define SAMPLER_MIN_FILTER_LINEAR_MIPMAP_NEAREST 9985
#define SAMPLER_MIN_FILTER_NEAREST_MIPMAP_LINEAR 9986
#define SAMPLER_MIN_FILTER_LINEAR_MIPMAP_LINEAR 9987

#define SAMPLER_WRAPS_MIRROR_REPEAT 33648
#define SAMPLER_WRAPS_REPEAT 10497
#define SAMPLER_WRAPS_CLAMP_TO_EDGE 33071
#define SAMPLER_WRAPT_MIRROR_REPEAT 33648
#define SAMPLER_WRAPT_REPEAT 10497
#define SAMPLER_WRAPT_CLAMP_TO_EDGE 33071

#define BUFFERVIEW_TARGET_ARRAY_BUFFER 34962
#define BUFFERVIEW_TARGET_ELEMENT_ARRAY_BUFFER 34963

#define ACCESOR_COMPONENT_TYPE_SIGNED_BYTE 5120     // 8 bits
#define ACCESOR_COMPONENT_TYPE_UNSIGNED_BYTE 5121   // 8 bits
#define ACCESOR_COMPONENT_TYPE_SHORT 5122           // 16 bits
#define ACCESOR_COMPONENT_TYPE_UNSIGNED_SHORT 5123  // 16 bits
#define ACCESOR_COMPONENT_TYPE_UNSIGNED_INT 5125    // 32 bits
#define ACCESOR_COMPONENT_TYPE_FLOAT 5126           // 32 bits

#define ACCESOR_TYPE_SCALAR "SCALAR"  // 1 component
#define ACCESOR_TYPE_VEC2 "VEC2"      // 2 components
#define ACCESOR_TYPE_VEC3 "VEC3"      // 3 components
#define ACCESOR_TYPE_VEC4 "VEC4"      // 4 components
#define ACCESOR_TYPE_MAT2 "MAT2"      // 4 components
#define ACCESOR_TYPE_MAT3 "MAT3"      // 9 components
#define ACCESOR_TYPE_MAT4 "MAT4"      // 16 components

#define PRIMITIVE_MODE_POINTS 0
#define PRIMITIVE_MODE_LINES 1
#define PRIMITIVE_MODE_LINE_LOOP 2
#define PRIMITIVE_MODE_LINE_STRIP 3
#define PRIMITIVE_MODE_TRIANGLES 4
#define PRIMITIVE_MODE_TRIANGLE_STRIP 5
#define PRIMITIVE_MODE_TRIANGLE_FAN 6

// Node �N��@�Ӫ���A�� RTS �T���ݩʡA�B Node �]�t�@�� vector �s���L Node ��
// index�A�o�� index �N�O Sub-Node
struct Node {
  std::string name;
  std::vector<int> children_index;
  int mesh_index;

  int skin_index;
  std::vector<int> weights;

  float translation[3];
  float rotation[4];  // w, x, y, z
  float scale[3];

  int camera_index;
  std::vector<float> matrix;

  // extensions
  // extras
};

struct Accessor {
  std::string name;
  int buffer_view_index;
  int byte_offset;     // default 0
  int component_type;  // required
  int count;           // required
  bool normalized;     // default false
  std::string type;    // required

  std::vector<float> max;
  std::vector<float> min;

  // extensions
  // extras
};

struct Asset {
  std::string generator;
  std::string copyright;  // required
  std::string version;
  std::string minVersion;

  // extensions
  // extras
};

struct Buffer {
  std::string name;
  std::string uri;
  int byte_length;  // required

  // extensions
  // extras
};

// �C�� int ���O���V bufferView �� index
struct Attribute {
  int POSITION = -1;
  int TEXCOORD_0 = -1;
  int NORMAL = -1;
  int TANGENT = -1;
};

// attributes �O���V accessor �� index
struct Primitive {
  Attribute attributes;
  int mode = 4;  // default 4
  int indices_index = -1;
  int material_index = -1;
};

struct Mesh {
  std::vector<Primitive> primitives;
};

struct BufferView {
  std::string name;
  int buffer_index;
  int byte_offset;  // �b bin ���������q
  int byte_length;  // ���h�� byte
  int byte_stride;  // �@�Ӥ����� byte ����
  int target;

  // extensions
  // extras
};

struct MatPBRMetallicRoughness {
  float base_color_factor[4];
  int base_color_texture_index;
  int metallic_roughness_texture_index;
  float metallic_factor;
};

struct Material {
  std::string name;

  float base_color[3];
  float alpha_cutoff;
  std::string alpha_mode;
  bool double_sided;
  unsigned int normal_texture_index;

  MatPBRMetallicRoughness pbr_metallic_roughness;
};

struct Scene {
  std::string name;
  std::vector<unsigned int> nodes_index;  // ���V���U�� Node
};

// source �N�O image �� index
struct Texture {
  int sampler_index;
  int source;
  std::string name;

  // extensions
  // extras
};

// �x�s�Ϥ������
struct Image {
  // gltf ����T
  std::string mimeType;
  std::string uri;

  // �Ϥ�����ڸ��
  int height;
  int width;
  int nrChannels;
  std::vector<unsigned char> data;
};

#define SAMPLER 00  // �����Ψөw�q���P�ت� mag_filter
struct Sampler {
  unsigned int mag_filter;
  unsigned int min_filter;
  unsigned int wrap_s;
  unsigned int wrap_t;
};

class GltfLoader {
 public:
  GltfLoader() = default;
  // Ū�X gltf �ɮת����e�A�åB�ন JSON ��ѪR
  GltfLoader(const std::string& path) {
    file_path_ = path;

    std::ifstream ifs(path, std::ios::binary);
    std::string content;
    if (ifs) {
      ifs.seekg(0, std::ios::end);            // Ū�����в��ʨ�̫�
      content.resize(ifs.tellg());            // Ū������
      ifs.seekg(0, std::ios::beg);            // Ū�����в��ʨ�̫e
      ifs.read(&content[0], content.size());  // Ū�����e
      ifs.close();
    }

    json_ = json::parse(content);

    asset_ = getAsset();
    accessors_ = getAcessors();
    buffer_views_ = getBufferViews();
    samplers_ = getSamplers();
    textures_ = getTextures();
    images_ = getImages();
    buffers_ = getBuffers();
    materials_ = getMaterials();
    meshes_ = getMeshes();
    nodes_ = getNodes();
    scenes_ = getScenes();
    default_scene_ = getDefaultScene();

    data_ = loadData();
    // loadImage();  // TODO: �ϥΦh�����
    loadMesh();
  }
  std::vector<OutputPrimitives>& getOutputPrimitives() {
    return output_primitives_;
  }

 public:
  void loadMesh() {
    // Mesh �̭����\�h primative�A�C�� primative �]���� attribute
    // �ڭn�ϥ� primitive �� attribute �Ө��o accessor �����
    // accessor �i�D�ڭ̸�Ʀb bufferView �����̡A�H�θ�ƪ����A

    for (size_t i = 0; i < meshes_.size(); i++) {
      output_primitives_ = loadPrimitives(meshes_[i]);
    }
  }

  std::vector<OutputPrimitives> loadPrimitives(Mesh& mesh) {
    std::vector<OutputPrimitives> results;
    for (int i = 0; i < mesh.primitives.size(); i++) {
      OutputPrimitives output_primitives;
      loadVertexBuffer(output_primitives, mesh.primitives[i]);
      loadIndexBuffer(output_primitives, mesh.primitives[i]);
      loadMode(output_primitives, mesh.primitives[i]);
      // loadMaterials();
      results.push_back(output_primitives);
    }
    return results;
  }

  void loadMode(OutputPrimitives& result, Primitive& primitives) {
    switch (primitives.mode) {
      case PRIMITIVE_MODE_POINTS:
        result.mode = PrimitiveMode::kPoints;
        break;
      case PRIMITIVE_MODE_LINES:
        result.mode = PrimitiveMode::kLines;
        break;
      case PRIMITIVE_MODE_LINE_LOOP:
        result.mode = PrimitiveMode::kLineLoop;
        break;
      case PRIMITIVE_MODE_LINE_STRIP:
        result.mode = PrimitiveMode::kLineStrip;
        break;
      case PRIMITIVE_MODE_TRIANGLES:
        result.mode = PrimitiveMode::kTriangles;
        break;
      case PRIMITIVE_MODE_TRIANGLE_STRIP:
        result.mode = PrimitiveMode::kTriangleStrip;
        break;
      case PRIMITIVE_MODE_TRIANGLE_FAN:
        result.mode = PrimitiveMode::kTriangleFan;
        break;
      default:
        throw std::invalid_argument("Unknown primitive mode: " +
                                    std::to_string(primitives.mode));
    }
  }
  void loadVertexBuffer(OutputPrimitives& result, Primitive& primitives) {
    int pos_accessor_index = primitives.attributes.POSITION;
    int normal_accessor_index = primitives.attributes.NORMAL;
    int uv_accessor_index = primitives.attributes.TEXCOORD_0;
    int tangent_accessor_index = primitives.attributes.TANGENT;

    Accessor* pos_accessor = nullptr;
    Accessor* normal_accessor = nullptr;
    Accessor* uv_accessor = nullptr;
    Accessor* tangent_accessor = nullptr;

    if (pos_accessor_index != -1) {
      pos_accessor = &accessors_[pos_accessor_index];
      result.positions = loadDataByAccessor(*pos_accessor);
      result.pos_comp = getComponentCount(pos_accessor->type);
      result.pos_type = getComponentType(pos_accessor->component_type);
    }
    if (normal_accessor_index != -1) {
      normal_accessor = &accessors_[normal_accessor_index];
      result.normals = loadDataByAccessor(*normal_accessor);
      result.normal_comp = getComponentCount(normal_accessor->type);
      result.normal_type = getComponentType(normal_accessor->component_type);
    }
    if (uv_accessor_index != -1) {
      uv_accessor = &accessors_[uv_accessor_index];
      result.uvs = loadDataByAccessor(*uv_accessor);
      result.uv_comp = getComponentCount(uv_accessor->type);
      result.uv_type = getComponentType(uv_accessor->component_type);
    }
    if (tangent_accessor_index != -1) {
      tangent_accessor = &accessors_[tangent_accessor_index];
      result.tangents = loadDataByAccessor(*tangent_accessor);
      result.tan_comp = getComponentCount(tangent_accessor->type);
      result.tan_type = getComponentType(tangent_accessor->component_type);
    }
  }

  void loadIndexBuffer(OutputPrimitives& result, Primitive& primitives) {
    int indices_index = primitives.indices_index;
    if (indices_index != -1) {
      Accessor* indices_accessor = &accessors_[indices_index];
      result.indices = loadDataByAccessor(*indices_accessor);
      result.indices_type = getComponentType(indices_accessor->component_type);
      result.use_indices = true;
    } else {
      result.use_indices = false;
    }
  }

  void loadImage() {
    for (unsigned int i = 0; i < images_.size(); i++) {
      std::string file_dir =
          file_path_.substr(0, file_path_.find_last_of("/") + 1);

      auto& img = images_[i];
      std::string path = file_dir + img.uri;
      stbi_uc* data =
          stbi_load(path.c_str(), &img.width, &img.height, &img.nrChannels, 0);
      if (data) {
        img.data.resize(img.width * img.height * img.nrChannels);
        memcpy(img.data.data(), data, img.width * img.height * img.nrChannels);
        stbi_image_free(data);
      }
    }
  }

  // scene ���\�h�ӡA�� gltf �|�Х� default scene �� index
  unsigned int getDefaultScene() { return json_["scene"]; }

  Asset getAsset() {
    Asset asset;
    asset.version = json_["asset"].value("version", "");
    asset.generator = json_["asset"].value("generator", "");
    return asset;
  }

  std::vector<Scene> getScenes() {
    std::vector<Scene> scenes;
    for (unsigned int i = 0; i < json_["scenes"].size(); i++) {
      Scene scene;
      scene.name = json_["scenes"][i].value("name", "");
      for (unsigned int j = 0; j < json_["scenes"][i]["nodes"].size(); j++) {
        scene.nodes_index.push_back(json_["scenes"][i]["nodes"][j]);
      }
      scenes.push_back(scene);
    }
    return scenes;
  }

  std::vector<Node> getNodes() {
    std::vector<Node> results;
    for (unsigned int i = 0; i < json_["nodes"].size(); i++) {
      Node node;
      node.name = json_["nodes"][i].value("name", "");

      // ���o translation �����
      if (json_["nodes"][i].contains("translation")) {
        auto translation_array = json_["nodes"][i]["translation"];
        if (translation_array.is_array() && translation_array.size() == 3) {
          node.translation[0] = translation_array[0].get<float>();
          node.translation[1] = translation_array[1].get<float>();
          node.translation[2] = translation_array[2].get<float>();
        }
      } else {
        node.translation[0] = 0;
        node.translation[1] = 0;
        node.translation[2] = 0;
      }

      // ���o rotation �����
      if (json_["nodes"][i].contains("rotation")) {
        auto rotation_array = json_["nodes"][i]["rotation"];
        if (rotation_array.is_array() && rotation_array.size() == 4) {
          node.rotation[0] = rotation_array[0].get<float>();
          node.rotation[1] = rotation_array[1].get<float>();
          node.rotation[2] = rotation_array[2].get<float>();
          node.rotation[3] = rotation_array[3].get<float>();
        }
      } else {
        node.rotation[0] = 0;
        node.rotation[0] = 0;
        node.rotation[0] = 0;
        node.rotation[0] = 0;
      }

      // ���o scale �����
      if (json_["nodes"][i].contains("scale")) {
        auto scale_array = json_["nodes"][i]["scale"];
        if (scale_array.is_array() && scale_array.size() == 3) {
          node.scale[0] = scale_array[0].get<float>();
          node.scale[1] = scale_array[1].get<float>();
          node.scale[2] = scale_array[2].get<float>();
        }
      } else {
        node.scale[0] = 0;
        node.scale[1] = 0;
        node.scale[2] = 0;
      }

      // ���o camera �� index
      if (json_["nodes"][i].contains("camera")) {
        node.camera_index = json_["nodes"][i].value("camera", -1);
      }

      // ���o matrix �����
      if (json_["nodes"][i].contains("matrix")) {
        auto matrix_array = json_["nodes"][i]["matrix"];
        if (matrix_array.is_array() && matrix_array.size() == 16) {
          for (unsigned int j = 0; j < 16; j++) {
            node.matrix.push_back(matrix_array[j].get<float>());
          }
        }
      }

      // ���o mesh �� index
      if (json_["nodes"][i].contains("mesh")) {
        node.mesh_index = json_["nodes"][i].value("mesh", -1);
      }

      results.push_back(node);
    }
    return results;
  }

  std::vector<Buffer> getBuffers() {
    std::vector<Buffer> buffers;
    for (unsigned int i = 0; i < json_["buffers"].size(); i++) {
      Buffer buffer;
      buffer.uri = json_["buffers"][i].value("uri", "");
      buffer.byte_length = json_["buffers"][i].value("byteLength", 0);
      buffers.push_back(buffer);
    }
    return buffers;
  }

  std::vector<BufferView> getBufferViews() {
    std::vector<BufferView> buffer_views;
    for (unsigned int i = 0; i < json_["bufferViews"].size(); i++) {
      BufferView buffer_view;
      buffer_view.buffer_index = json_["bufferViews"][i].value("buffer", -1);
      buffer_view.byte_offset = json_["bufferViews"][i].value("byteOffset", 0);
      buffer_view.byte_length = json_["bufferViews"][i].value("byteLength", 0);
      buffer_view.byte_stride = json_["bufferViews"][i].value("byteStride", 1);
      buffer_view.target = json_["bufferViews"][i].value("target", 0);
      buffer_views.push_back(buffer_view);
    }
    return buffer_views;
  }

  std::vector<Sampler> getSamplers() {
    std::vector<Sampler> samplers;
    for (unsigned int i = 0; i < json_["samplers"].size(); i++) {
      Sampler sampler;
      sampler.mag_filter = json_["samplers"][i].value("magFilter", 9729);
      sampler.min_filter = json_["samplers"][i].value("minFilter", 9987);
      sampler.wrap_s = json_["samplers"][i].value("wrapS", 10497);
      sampler.wrap_t = json_["samplers"][i].value("wrapT", 10497);
      samplers.push_back(sampler);
    }
    return samplers;
  }

  std::vector<Mesh> getMeshes() {
    std::vector<Mesh> meshes;
    for (unsigned int i = 0; i < json_["meshes"].size(); i++) {
      Mesh mesh;
      for (unsigned int j = 0; j < json_["meshes"][i]["primitives"].size();
           j++) {
        Primitive primitive;
        primitive.attributes =
            getAttribute(json_["meshes"][i]["primitives"][j]["attributes"]);
        primitive.indices_index =
            json_["meshes"][i]["primitives"][j].value("indices", -1);
        primitive.material_index =
            json_["meshes"][i]["primitives"][j].value("material", -1);
        primitive.mode = json_["meshes"][i]["primitives"][j].value("mode", 4);
        mesh.primitives.push_back(primitive);
      }
      meshes.push_back(mesh);
    }
    return meshes;
  }

  // ���o .bin �����
  std::vector<char> loadData() {
    std::string byte_text;
    std::string uri =
        json_["buffers"][0]["uri"];  // ���o .bin �����|�A�o�O�G�i�쪺

    std::string file_dir =
        file_path_.substr(0, file_path_.find_last_of("/") + 1);
    std::string bin_path = file_dir + "/" + uri;

    std::ifstream ifs(bin_path, std::ios::binary);

    std::vector<char> data;
    if (ifs) {
      ifs.seekg(0, std::ios::end);      // Ū�����в��ʨ�̫�
      data.resize(ifs.tellg());         // Ū������
      ifs.seekg(0, std::ios::beg);      // Ū�����в��ʨ�̫e
      ifs.read(&data[0], data.size());  // Ū�����e
      ifs.close();
    }

    return data;
  }

  std::vector<Accessor> getAcessors() {
    std::vector<Accessor> accessors;
    for (unsigned int i = 0; i < json_["accessors"].size(); i++) {
      Accessor accessor;
      accessor.buffer_view_index = json_["accessors"][i].value("bufferView", 0);
      accessor.byte_offset = json_["accessors"][i].value("byteOffset", 0);
      accessor.component_type = json_["accessors"][i].value("componentType", 0);
      accessor.count = json_["accessors"][i].value("count", 0);
      accessor.type = json_["accessors"][i].value("type", "");
      accessors.push_back(accessor);
    }
    return accessors;
  }

  // accessor.type ���h�֭� component
  int getComponentCount(const std::string& accessor_type) {
    if (accessor_type == ACCESOR_TYPE_SCALAR) {
      return 1;
    } else if (accessor_type == ACCESOR_TYPE_VEC2) {
      return 2;
    } else if (accessor_type == ACCESOR_TYPE_VEC3) {
      return 3;
    } else if (accessor_type == ACCESOR_TYPE_VEC4) {
      return 4;
    } else if (accessor_type == ACCESOR_TYPE_MAT2) {
      return 4;
    } else if (accessor_type == ACCESOR_TYPE_MAT3) {
      return 9;
    } else if (accessor_type == ACCESOR_TYPE_MAT4) {
      return 16;
    } else {
      throw std::invalid_argument("Unknown accessor type: " + accessor_type);
    }
  }

  // accessor.component_type �@�� component ���X�� byte
  int getComponentByte(int component_type) {
    if (component_type == ACCESOR_COMPONENT_TYPE_SIGNED_BYTE ||
        component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_BYTE) {
      return 1;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_SHORT ||
               component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_SHORT) {
      return 2;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_INT ||
               component_type == ACCESOR_COMPONENT_TYPE_FLOAT) {
      return 4;
    } else {
      throw std::invalid_argument("Unknown component type: " +
                                  std::to_string(component_type));
    }
  }

  DataType getComponentType(int component_type) {
    if (component_type == ACCESOR_COMPONENT_TYPE_SIGNED_BYTE) {
      return DataType::kByte;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_BYTE) {
      return DataType::kUnsignedByte;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_SHORT) {
      return DataType::kShort;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_SHORT) {
      return DataType::kUnsignedShort;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_UNSIGNED_INT) {
      return DataType::kUnsignedInt;
    } else if (component_type == ACCESOR_COMPONENT_TYPE_FLOAT) {
      return DataType::kFloat;
    } else {
      throw std::invalid_argument("Unknown component type: " +
                                  std::to_string(component_type));
    }
  }

  // �z�L accessor ���o�Y�@�� attribute �����e�Abuffer ���O little endian
  std::vector<uint8_t> loadDataByAccessor(Accessor& accessor) {
    std::vector<uint8_t> results;
    // attribute ���`��ƶq
    int lenght = accessor.count * getComponentCount(accessor.type) *
                 getComponentByte(accessor.component_type);
    results.resize(lenght);

    BufferView buffer_view = buffer_views_[accessor.buffer_view_index];
    // acc_byte_offset �O accessor �������q�Abyte_offset �O bufferView �������q
    // �|�ݭn�o��Ӱ����q�O�]���i�঳�h�� accessor �@�ΦP�@��
    // bufferView�A�۷��@�� Vertex �P�ɦ� Position, Normal, UV ����T
    // �ӯu���������q�O�o��Ӱ����q�ۥ[
    int beginning_offset = buffer_view.byte_offset + accessor.byte_offset;
    // �`��ƪ���: �@�� vertex ���X�� component * �@�� component ���X�� byte *
    // ���h�֭� vertex

    if (buffer_view.byte_stride == 0) {  // �P�@�� attribute �s��s��
      memcpy(results.data(), data_.data() + beginning_offset, lenght);
    } else {  // �P�@�� attribute ���s��s��
      for (int i = 0; i < lenght; i += buffer_view.byte_stride) {
        memcpy(results.data() + i, data_.data() + beginning_offset + i,
               buffer_view.byte_stride);
      }
    }

    return results;
  }

  std::vector<Image> getImages() {
    std::vector<Image> results;
    for (unsigned int i = 0; i < json_["images"].size(); i++) {
      Image image;
      image.uri = json_["images"][i]["uri"];
      image.mimeType = json_["images"][i]["mimeType"];
      results.push_back(image);
    }
    return results;
  }

  std::vector<Texture> getTextures() {
    std::vector<Texture> textures;
    for (unsigned int i = 0; i < json_["textures"].size(); i++) {
      Texture texture;
      if (json_["textures"][i].contains("name")) {
        texture.name = json_["textures"][i]["name"];
      }
      if (json_["textures"][i].contains("sampler")) {
        texture.sampler_index = json_["textures"][i]["sampler"];
      }
      if (json_["textures"][i].contains("source")) {
        texture.source = json_["textures"][i]["source"];
      }
      textures.push_back(texture);
    }
    return textures;
  }

  std::vector<Material> getMaterials() {
    std::vector<Material> materials;
    for (unsigned int i = 0; i < json_["materials"].size(); i++) {
      Material material;

      // material.name = json_["materials"][i].value("name", "");
      // material.base_color = glm::vec3(1.0f);
      material.alpha_mode = json_["materials"][i].value("alphaMode", "OPAQUE");
      material.alpha_cutoff = json_["materials"][i].value("alphaCutoff", 0.5f);
      material.double_sided = json_["materials"][i].value("doubleSided", false);

      if (json_["materials"][i].contains("normalTexture")) {
        json_["materials"][i]["normalTexture"].value("index", -1);
      }

      // ���o PBRMetallicRoughness �����
      if (json_["materials"][i].contains("pbrMetallicRoughness")) {
        material.pbr_metallic_roughness = getMatPBRMetallicRoughness(
            json_["materials"][i]["pbrMetallicRoughness"]);
      }
      materials.push_back(material);
    }
    return materials;
  }

  // �H�U���O Material ���l�����
  MatPBRMetallicRoughness getMatPBRMetallicRoughness(
      json pbr_metallic_roughness) {
    MatPBRMetallicRoughness result;

    // ���o baseColorFactor �����
    if (pbr_metallic_roughness.contains("baseColorFactor")) {
      auto base_color_factor_array = pbr_metallic_roughness["baseColorFactor"];
      if (base_color_factor_array.is_array() &&
          base_color_factor_array.size() == 4) {
        result.base_color_factor[0] = base_color_factor_array[0].get<float>();
        result.base_color_factor[1] = base_color_factor_array[1].get<float>();
        result.base_color_factor[2] = base_color_factor_array[2].get<float>();
        result.base_color_factor[3] = base_color_factor_array[3].get<float>();
      }
    }

    if (pbr_metallic_roughness.contains("baseColorTexture")) {
      result.base_color_texture_index =
          pbr_metallic_roughness["baseColorTexture"].value("index", -1);
    }

    if (pbr_metallic_roughness.contains("metallicRoughnessTexture")) {
      result.base_color_texture_index =
          pbr_metallic_roughness["metallicRoughnessTexture"].value("index", -1);
    }

    if (pbr_metallic_roughness.contains("metallicFactor")) {
      result.metallic_factor = pbr_metallic_roughness["metallicFactor"];
    }

    return result;
  }

  // ���o primitive �� attribute
  Attribute getAttribute(json attributes) {
    Attribute result;
    for (json::iterator it = attributes.begin(); it != attributes.end(); ++it) {
      if (it.key() == "POSITION") {
        result.POSITION = it.value();
      } else if (it.key() == "TEXCOORD_0") {
        result.TEXCOORD_0 = it.value();
      } else if (it.key() == "NORMAL") {
        result.NORMAL = it.value();
      } else if (it.key() == "TANGENT") {
        result.TANGENT = it.value();
      }
    }
    return result;
  }

  std::vector<Accessor> getAccessors() {
    std::vector<Accessor> accessors;
    for (unsigned int i = 0; i < json_["accessors"].size(); i++) {
      Accessor accessor;
      accessor.type = json_["accessors"][i].value("type", "");
      accessor.component_type = json_["accessors"][i].value("componentType", 0);
      accessor.count = json_["accessors"][i].value("count", 0);
      accessor.buffer_view_index = json_["accessors"][i].value("bufferView", 0);
      accessor.byte_offset = json_["accessors"][i].value("byteOffset", 0);
      accessors.push_back(accessor);
    }
    return accessors;
  }

 private:
  unsigned int default_scene_ = 0;
  Asset asset_;
  std::vector<Accessor> accessors_;
  std::vector<Scene> scenes_;
  std::vector<Node> nodes_;
  std::vector<BufferView> buffer_views_;
  std::vector<Image> images_;
  std::vector<Texture> textures_;
  std::vector<Sampler> samplers_;
  std::vector<Mesh> meshes_;
  std::vector<Buffer> buffers_;
  std::vector<Material> materials_;

  std::vector<OutputPrimitives> output_primitives_;

  std::string file_path_;
  std::vector<char> data_;  // �x�s���I�y�СB�k�V�q�BUV �����
  json json_;
};

}  // namespace gltf

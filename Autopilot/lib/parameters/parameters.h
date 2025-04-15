#ifndef LIB_PARAMETERS_PARAMETERS_H_
#define LIB_PARAMETERS_PARAMETERS_H_

#include <stdint.h>
#include <stdbool.h>

// Type tags for template specialization
typedef enum {
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_INT32
} param_type_t;

// Base parameter class
struct ParamBase {
    const char* name;
    uint32_t hash;
    param_type_t type;
    bool is_set;
};

// Template parameter class
template<typename T, param_type_t PT>
struct Param : public ParamBase {
    T value;
    const T default_value;

    constexpr Param(const char* n, T def) :
        ParamBase{n, hash_string(n), PT, false},
        default_value(def) {
        value = def;
    }

    operator T() const { return get(); }
    T get() const { return value; }
    void set(T v) { value = v; is_set = true; }

private:
    static constexpr uint32_t hash_string(const char* str) {
        uint32_t h = 5381;
        while (*str) {
            h = ((h << 5) + h) + *str++;
        }
        return h;
    }
};

// Concrete types for easier declaration
typedef Param<float, PARAM_TYPE_FLOAT> ParamFloat;
typedef Param<int32_t, PARAM_TYPE_INT32> ParamInt;

// Parameter system interface
void param_register(ParamBase* param);
ParamBase* param_find(const char* name);
bool param_all_set(void);

#endif /* LIB_PARAMETERS_PARAMETERS_H_ */

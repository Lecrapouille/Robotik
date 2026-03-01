# Prologot API Reference

Complete API documentation for the Prologot C++ integration (namespace `robotik::prolog`).

## Quick Start

```cpp
#include <Robotik/Core/Prolog/Prologot.hpp>

using namespace robotik::prolog;

int main()
{
    Prologot prolog;

    // Initialize the Prolog engine
    prolog.initialize();

    // Load Prolog code from a string
    prolog.consult_string(R"(
        parent(tom, bob).
        parent(bob, ann).
        grandparent(X, Z) :- parent(X, Y), parent(Y, Z).
    )");

    // Query: check if a fact exists
    bool exists = prolog.query("parent(tom, bob)");  // true

    // Query: get first solution
    auto result = prolog.query_one("parent", {String("X"), String("Y")});

    // Query: get all solutions
    auto results = prolog.query_all("parent", {String("X"), String("Y")});

    // Dynamic facts
    prolog.add_fact("enemy(goblin, 10)");
    prolog.retract_fact("enemy(goblin, 10)");

    // Cleanup
    prolog.cleanup();

    return 0;
}
```

---

## Prologot Class

The main class providing SWI-Prolog integration for C++. This class wraps the SWI-Prolog C API.

### Type Aliases

```cpp
namespace robotik::prolog {
    using String = std::string;
    using Array = std::vector<std::any>;
    using Variant = std::any;
    using Dictionary = std::unordered_map<std::string, std::any>;
}
```

### Initialization and Cleanup

#### `initialize(options: Dictionary = {}) -> bool`

Initializes the SWI-Prolog engine with optional configuration.

This method performs the following steps:

1. Checks if already initialized (idempotent).
2. Parses the options Dictionary for configuration settings.
3. Sets up the SWI-Prolog home directory if provided.
4. Initializes the Prolog engine with the specified options.
5. Bootstraps helper predicates needed for `consult_string()`.

**Parameters:**

- `options` (Dictionary, optional): Configuration options (see below).

**Returns:** `true` if initialization succeeded, `false` otherwise.

**Initialization Options:**

**Main options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `"home"` | String | "" | Path to SWI-Prolog installation |
| `"quiet"` | bool | true | Suppress informational messages |
| `"goal"` | String/Array | - | Goal(s) to execute at startup |
| `"toplevel"` | String | "" | Custom toplevel goal |
| `"init file"` | String | "" | User initialization file |
| `"script file"` | String | "" | Script source file to load |

**Performance options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `"stack limit"` | String | "" | Prolog stack limit (e.g. "1g", "512m", "256k") |
| `"table space"` | String | "" | Space for SLG tables (e.g. "128m") |
| `"shared table space"` | String | "" | Space for shared SLG tables |
| `"optimized"` | bool | false | Enable optimized compilation |

**Behavior options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `"traditional"` | bool | false | Traditional mode, disable SWI-Prolog v7 extensions |
| `"threads"` | bool | true | Allow threads |
| `"packs"` | bool | true | Attach add-ons/packages |

**Error handling options:**

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `"on error"` | String | "print" | Error handling style: "print", "halt", "status" |
| `"on warning"` | String | "print" | Warning handling style: "print", "halt", "status" |

**Usage examples:**

```cpp
// Simple initialization (default, quiet mode)
prolog.initialize();

// Simple initialization with path to SWI-Prolog installation
Dictionary opts;
opts["home"] = String("/usr/lib/swipl");
prolog.initialize(opts);

// With verbose output
opts["quiet"] = false;
prolog.initialize(opts);

// Optimal configuration for AI reasoning
Dictionary ai_opts;
ai_opts["quiet"] = true;
ai_opts["optimized"] = true;
ai_opts["stack limit"] = String("2g");
ai_opts["table space"] = String("512m");
ai_opts["threads"] = true;
ai_opts["on error"] = String("print");
prolog.initialize(ai_opts);
```

#### `cleanup() -> void`

Cleans up and shuts down the Prolog engine.

This method is safe to call multiple times. It only performs cleanup if the engine was actually initialized. After cleanup, the engine must be re-initialized before use.

#### `is_initialized() -> bool`

Checks if the Prolog engine is currently initialized.

**Returns:** `true` if initialized and ready to use, `false` otherwise.

#### `get_last_error() -> String`

Gets the last error message from Prolog.

**Returns:** The last error message, or empty string if no error.

---

### File and Code Loading

#### `consult_file(filename: String) -> bool`

Loads a Prolog file into the knowledge base.

**Note:** Multiple calls to `consult_file()` and `consult_string()` accumulate clauses in the knowledge base.

**Parameters:**

- `filename` (String): Path to the Prolog file (.pl) to load.

**Returns:** `true` if the file was loaded successfully, `false` otherwise.

**Example:**

```cpp
prolog.consult_file("/path/to/family.pl");
prolog.consult_file("/path/to/game_rules.pl");
```

#### `consult_string(code: String) -> bool`

Loads Prolog code from a string into the knowledge base.

**Parameters:**

- `code` (String): The Prolog code to load (can be multi-line).

**Returns:** `true` if the code was loaded successfully, `false` otherwise.

**Example:**

```cpp
prolog.consult_string(R"(
    parent(tom, bob).
    parent(bob, ann).
    grandparent(X, Z) :- parent(X, Y), parent(Y, Z).
)");
```

---

### Query Execution

#### Prolog Variable Naming

In Prolog, variable names must start with an uppercase letter or underscore (`_`). Lowercase names are atoms (constants), not variables.

- `X`, `Y`, `Player`, `Enemy` → **Variables** (will be bound to values).
- `x`, `y`, `tom`, `bob` → **Atoms** (constant values).

#### `query(predicate: String, args: Array = {}) -> bool`

Executes a Prolog query and checks if it succeeds.

**Parameters:**

- `predicate` (String): The Prolog predicate name or full goal.
- `args` (Array, optional): Optional array of variable names or values.

**Returns:** `true` if the query succeeds, `false` otherwise.

**Example:**

```cpp
// Check if a fact exists
prolog.query("parent(tom, bob)");  // Returns true

// Check if a predicate has solutions
prolog.query("parent", {String("tom"), String("X")});  // Returns true if tom has children
```

#### `query_all(predicate: String, args: Array = {}) -> Array`

Executes a Prolog query and returns all solutions.

**Return format:**

- If `args` contains variable names: Array of Dictionary entries mapping variable names to values.
- Otherwise: Array of Variants representing solutions.

**Parameters:**

- `predicate` (String): The Prolog predicate name or full goal.
- `args` (Array, optional): Optional array of variable names or values.

**Returns:** Array of solutions. Empty array if no solutions.

**Example:**

```cpp
// Get all solutions (legacy format)
auto results = prolog.query_all("parent(X, Y)");
// Returns: Array of Dictionary with {"functor": "parent", "args": [...]}

// Get all solutions with variable extraction
auto results = prolog.query_all("parent", {String("X"), String("Y")});
// Returns: Array of Dictionary with {"X": value, "Y": value}
```

#### `query_one(predicate: String, args: Array = {}) -> Variant`

Executes a Prolog query and returns the first solution.

**Parameters:**

- `predicate` (String): The Prolog predicate name or full goal.
- `args` (Array, optional): Optional array of variable names or values.

**Returns:** The solution as Variant, or empty Variant if no solution.

**Example:**

```cpp
auto result = prolog.query_one("parent(tom, X)");
auto result = prolog.query_one("parent", {String("tom"), String("X")});
```

---

### Dynamic Facts

#### `add_fact(fact: String) -> bool`

Adds a fact into the Prolog knowledge base.

**Parameters:**

- `fact` (String): The Prolog fact to add (e.g., "likes(john, pizza)").

**Returns:** `true` if the fact was added successfully, `false` otherwise.

**Example:**

```cpp
prolog.add_fact("parent(tom, bob)");
prolog.add_fact("game_state(level, 5)");
```

#### `retract_fact(fact: String) -> bool`

Removes a fact from the Prolog knowledge base.

**Parameters:**

- `fact` (String): The Prolog fact to remove.

**Returns:** `true` if a matching fact was found and removed, `false` otherwise.

**Example:**

```cpp
prolog.retract_fact("parent(tom, bob)");
```

#### `retract_all(functor: String) -> bool`

Retracts all facts matching a functor pattern.

**Parameters:**

- `functor` (String): The functor pattern to match (can contain variables).

**Returns:** `true` if any matching facts were retracted, `false` otherwise.

**Example:**

```cpp
prolog.retract_all("likes(_, _)");      // Remove all likes/2 facts
prolog.retract_all("parent(tom, _)");   // Remove all facts with tom as parent
```

---

### Predicate Manipulation

#### `call_predicate(name: String, args: Array) -> bool`

Calls a Prolog predicate with arguments.

**Parameters:**

- `name` (String): Name of the predicate to call.
- `args` (Array): Array of arguments to pass.

**Returns:** `true` if the predicate call succeeded, `false` otherwise.

**Example:**

```cpp
Array arr;
arr.push_back(3);
Array list;
list.push_back(1);
list.push_back(2);
list.push_back(3);
list.push_back(4);
arr.push_back(list);
prolog.call_predicate("member", arr);  // Returns true
```

#### `call_function(name: String, args: Array) -> Variant`

Calls a Prolog predicate and returns a result value.

**Parameters:**

- `name` (String): Name of the predicate to call.
- `args` (Array): Array of input arguments (result is the last argument).

**Returns:** The result value as Variant, or empty Variant if the call failed.

**Example:**

```cpp
Array arr;
Array list;
list.push_back(1);
list.push_back(2);
list.push_back(3);
arr.push_back(list);
auto len = prolog.call_function("length", arr);
// Returns: 3
```

---

### Introspection

#### `predicate_exists(name: String, arity: int) -> bool`

Checks if a predicate exists with the given arity.

**Parameters:**

- `name` (String): Name of the predicate to check.
- `arity` (int): Number of arguments.

**Returns:** `true` if the predicate exists, `false` otherwise.

**Example:**

```cpp
if (prolog.predicate_exists("parent", 2)) {
    std::cout << "parent/2 is defined" << std::endl;
}
```

#### `list_predicates() -> Array`

Lists all currently defined predicates.

**Returns:** Array of Dictionary objects describing each predicate.

---

## Type Conversion Reference

### Prolog Term → C++ Variant

| Prolog Term Type | C++ Type | Notes |
|------------------|----------|-------|
| `PL_VARIABLE` | empty `std::any` | Unbound variables |
| `PL_ATOM` | `std::string` | Prolog atoms become strings |
| `PL_INTEGER` | `int64_t` | 64-bit integers |
| `PL_FLOAT` | `double` | Double-precision floats |
| `PL_STRING` | `std::string` | Prolog strings |
| `PL_NIL` | empty `Array` | Empty list `[]` |
| `PL_LIST_PAIR` | `Array` | Prolog lists |
| `PL_TERM` (compound) | `Dictionary` | `{"functor": "name", "args": [...]}` |

### C++ Variant → Prolog Term

| C++ Type | Prolog Term Type | Notes |
|----------|------------------|-------|
| empty `std::any` | `PL_ATOM` (`[]`) | Null becomes empty list |
| `bool` | `PL_ATOM` | `true` or `false` |
| `int` / `int64_t` | `PL_INTEGER` | Integer values |
| `double` | `PL_FLOAT` | Float values |
| `std::string` | `PL_ATOM` | Strings become atoms |
| `Array` (empty) | `PL_NIL` | Empty list |
| `Array` (non-empty) | `PL_LIST_PAIR` | Prolog list |
| `Dictionary` | `PL_TERM` | Compound term |

---

## Use Cases for Robotics

### Behavior Tree Conditions

```cpp
prolog.consult_string(R"(
    can_grasp(Object) :-
        gripper_state(open),
        object_reachable(Object),
        object_graspable(Object).

    should_avoid(Object) :-
        object_type(Object, obstacle),
        distance(Object, D),
        D < 0.5.
)");

// Check conditions in behavior tree
if (prolog.query("can_grasp(target_object)")) {
    // Execute grasp action
}
```

### Rule-Based Decision Making

```cpp
prolog.consult_string(R"(
    next_action(grasp) :- target_visible, gripper_empty, target_reachable.
    next_action(approach) :- target_visible, gripper_empty, \+ target_reachable.
    next_action(search) :- \+ target_visible.
    next_action(place) :- \+ gripper_empty.
)");

auto action = prolog.query_one("next_action(Action)");
```

### Dynamic World State

```cpp
// Update world state
prolog.retract_all("robot_position(_, _)");
prolog.add_fact("robot_position(1.5, 2.3)");

prolog.retract_all("object_detected(_, _, _)");
for (auto& obj : detected_objects) {
    std::string fact = "object_detected(" + obj.id + ", " +
                       std::to_string(obj.x) + ", " +
                       std::to_string(obj.y) + ")";
    prolog.add_fact(fact);
}
```

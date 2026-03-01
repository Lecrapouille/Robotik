/*
 * MIT License
 * Copyright (c) 2024 Lecrapouille <lecrapouille@gmail.com>
 *
 * Prolog - SWI-Prolog integration for C++
 *
 * This file defines the main Prolog class that provides an interface
 * to the SWI-Prolog engine.
 */

#pragma once

#include <SWI-Prolog.h>
#include <any>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik::prolog {

using String = std::string;
using Variant = std::any;
using Array = std::vector<Variant>;
using Dictionary = std::unordered_map<std::string, Variant>;

/**
 * @class Prolog
 * @brief Main class providing SWI-Prolog integration for C++.
 *
 * This class wraps the SWI-Prolog C API and exposes it to C++,
 * allowing users to execute Prolog queries, assert/retract facts,
 * and consult Prolog files.
 */
class Prolog
{
public:

    /**
     * @brief Constructs a new Prolog instance.
     *
     * Initializes the singleton pointer and sets the initialized flag to false.
     * The Prolog engine is not started until initialize() is called.
     */
    Prolog();

    /**
     * @brief Destructs the Prolog instance.
     *
     * Ensures proper cleanup of the Prolog engine and resets the singleton
     * pointer. This prevents memory leaks and ensures clean shutdown.
     */
    ~Prolog();

    /**
     * @brief Gets the singleton instance of Prolog.
     *
     * This static method provides global access to the Prolog instance.
     * Useful for accessing Prolog from C++ code without passing references.
     *
     * @return Pointer to the singleton instance, or nullptr if not created yet.
     */
    static Prolog* get_singleton();

    // =========================================================================
    // Initialization and Cleanup
    // =========================================================================

    /**
     * @brief Initializes the SWI-Prolog engine with optional configuration.
     *
     * This method performs the following steps:
     * 1. Checks if already initialized (idempotent)
     * 2. Parses the options Dictionary for configuration settings
     * 3. Sets up the SWI-Prolog home directory if provided
     * 4. Initializes the Prolog engine with the specified options
     * 5. Bootstraps helper predicates needed for consult_string()
     *
     * The bootstrap predicates enable loading Prolog code from strings by:
     * - Parsing multi-line Prolog code into individual clauses
     * - Handling directives (:-) and queries (?-) appropriately
     * - Asserting regular clauses into the knowledge base
     *
     * @param p_options Dictionary containing initialization options:
     *   Main options:
     *     - "home" (String): Path to SWI-Prolog installation
     *     - "quiet" (bool): Suppress informational messages (default: true)
     *     - "goal" (String/Array): Goal(s) to execute at startup
     *     - "toplevel" (String): Custom toplevel goal
     *     - "init file" (String): User initialization file
     *     - "script file" (String): Script source file to load
     *   Performance options:
     *     - "stack limit" (String): Prolog stack limit (e.g. "1g", "512m")
     *     - "table space" (String): Space for SLG tables (e.g. "128m")
     *     - "shared table space" (String): Space for shared SLG tables
     *     - "optimized" (bool): Enable optimized compilation
     *   Behavior options:
     *     - "traditional" (bool): Traditional mode, disable v7 extensions
     *     - "threads" (bool): Allow threads (default: true)
     *     - "packs" (bool): Attach add-ons/packages (default: true)
     *   Error handling:
     *     - "on error" (String): Error handling style ("print", "halt",
     * "status")
     *     - "on warning" (String): Warning handling style ("print", "halt",
     * "status")
     *   Advanced options:
     *     - "prolog flags" (Dictionary): Define Prolog flags
     *     - "file search paths" (Dictionary): Define file search paths
     *     - "custom args" (Array): Additional custom arguments
     *
     * @return true if initialization succeeded, false otherwise.
     */
    bool initialize(Dictionary const& p_options = Dictionary());

    /**
     * @brief Cleans up and shuts down the Prolog engine.
     *
     * This method is safe to call multiple times. It only performs cleanup
     * if the engine was actually initialized. After cleanup, the engine
     * must be re-initialized before use.
     */
    void cleanup();

    /**
     * @brief Checks if the Prolog engine is currently initialized.
     *
     * @return true if initialized and ready to use, false otherwise.
     */
    bool is_initialized() const;

    // =========================================================================
    // File and Code Consultation
    // =========================================================================

    /**
     * @brief Consults a Prolog file into the knowledge base.
     *
     * This method uses Prolog's built-in consult/1 predicate to load a .pl
     * file. The file is parsed and all clauses are added to the knowledge base.
     *
     * Multiple calls to consult_file() and consult_string() accumulate clauses
     * in the knowledge base. Each new file or code string adds its clauses to
     * the existing knowledge base without removing previous ones. If you need
     * to replace the knowledge base, use retract_all() to remove specific
     * predicates first, or reinitialize the engine with cleanup() and
     * initialize().
     *
     * @param p_filename Path to the Prolog file (.pl) to load.
     * @return true if the file was loaded successfully, false otherwise.
     *
     * @example
     * prolog.consult_file("/path/to/family.pl")
     */
    bool consult_file(String const& p_filename);

    /**
     * @brief Consults Prolog code from a string into the knowledge base.
     *
     * This method uses the bootstrap predicate load_program_from_string/1
     * (created during initialization) to parse and load multi-line Prolog code.
     * The code can contain multiple clauses, directives, and queries.
     *
     * Multiple calls to consult_string() and consult_file() accumulate clauses
     * in the knowledge base. Each new code string adds its clauses to the
     * existing knowledge base without removing previous ones. If you need to
     * replace the knowledge base, use retract_all() to remove specific
     * predicates first, or reinitialize the engine with cleanup() and
     * initialize().
     *
     * @param p_prolog_code The Prolog code to load (can be multi-line).
     * @return true if the code was loaded successfully, false otherwise.
     *
     * @example
     * prolog.consult_string(R"(
     *     parent(tom, bob).
     *     parent(bob, ann).
     *     grandparent(X, Z) :- parent(X, Y), parent(Y, Z).
     * )")
     */
    bool consult_string(String const& p_prolog_code);

    // =========================================================================
    // Query Execution
    // =========================================================================

    /**
     * @brief Executes a Prolog query and checks if it succeeds.
     *
     * This method executes a query and returns true if at least one solution
     * exists. It does not collect or return the solutions themselves.
     *
     * Note: Do not include a trailing period ('.') in the query string. If a
     * period is present, it will be automatically removed. For example,
     * "parent(tom, bob)." will be treated as "parent(tom, bob)".
     *
     * @param p_predicate The Prolog predicate name (e.g., "parent") or full
     * goal (e.g., "member(X, [1,2,3])").
     * @param p_args Optional array of variable names (e.g., ["X", "Y"]) or
     * values. If empty, p_predicate is treated as a full goal.
     * @return true if the query succeeds (has at least one solution), false
     * otherwise.
     *
     * @example
     * prolog.query("parent(tom, bob)")  // Returns true
     * prolog.query("parent", {"tom", "X"})  // Returns true if tom has children
     */
    bool query(String const& p_predicate, Array const& p_args = Array());

    /**
     * @brief Executes a Prolog query and returns all solutions.
     *
     * This method uses Prolog's findall/3 to collect all solutions.
     *
     * Note: Do not include a trailing period ('.') in the query string. If a
     * period is present, it will be automatically removed.
     *
     * Return format:
     * - If p_args contains variable names: Array of Dictionary entries
     *   (e.g., {"X": value1, "Y": value2}).
     * - Otherwise: Array of Variants representing solutions, where each
     * solution may be:
     *     - a String (for atoms),
     *     - a Dictionary (for compound terms, e.g., {"functor": "name", "args":
     * [...]}),
     *     - or an Array (for Prolog lists).
     * - Anonymous variables ("_") appear as null in the results.
     *
     * @param p_predicate The Prolog predicate name (e.g., "parent") or full
     * goal.
     * @param p_args Optional array of variable names (e.g., ["X", "Y"]) or
     * values. If empty, p_predicate is treated as a full goal.
     * @return Array of solutions. Empty array if no solutions.
     *
     * @example
     * auto results = prolog.query_all("parent(X, Y)")
     * auto results = prolog.query_all("parent", {"X", "Y"})
     */
    Array query_all(String const& p_predicate, Array const& p_args = Array());

    /**
     * @brief Executes a Prolog query and returns the first solution.
     *
     * This method executes a query and returns only the first solution found.
     * Returns an empty Variant if no solution is found.
     *
     * Note: Do not include a trailing period ('.') in the query string. If a
     * period is present, it will be automatically removed.
     *
     * @param p_predicate The Prolog predicate name (e.g., "parent") or full
     * goal.
     * @param p_args Optional array of variable names (e.g., ["X", "Y"]) or
     * values. If empty, p_predicate is treated as a full goal.
     * @return The solution as Variant (or Dictionary if variables specified),
     * or empty Variant if no solution.
     *
     * @example
     * auto result = prolog.query_one("parent(tom, X)")
     * auto result = prolog.query_one("parent", {"tom", "X"})
     */
    Variant query_one(String const& p_predicate, Array const& p_args = Array());

    /**
     * @brief Gets the last error message from Prolog.
     *
     * This method returns the last error message stored in m_last_error.
     * Note: This method does not call push_error(). Errors are automatically
     * handled according to the "on error" and "on warning" options set during
     * initialization. Use this method to retrieve error messages for custom
     * error handling or logging.
     *
     * @return The last error message, or empty string if no error.
     */
    String get_last_error() const;

    // =========================================================================
    // Dynamic Assertions
    // =========================================================================

    /**
     * @brief Adds a fact into the Prolog knowledge base.
     *
     * This method adds a new clause to the knowledge base. The fact will be
     * available for subsequent queries. Uses Prolog's assert/1 predicate.
     *
     * Note: Do not include a trailing period ('.') in the fact string. If a
     * period is present, it will be automatically removed. For example,
     * "parent(tom, bob)." will be treated as "parent(tom, bob)".
     *
     * @param p_fact The Prolog fact to add (e.g., "likes(john, pizza)").
     * @return true if the fact was added successfully, false otherwise.
     *
     * @example
     * prolog.add_fact("parent(tom, bob)")
     * prolog.add_fact("game_state(level, 5)")
     */
    bool add_fact(String const& p_fact);

    /**
     * @brief Removes a fact from the Prolog knowledge base.
     *
     * This method removes a clause from the knowledge base. Uses Prolog's
     * retract/1 predicate, which removes the first matching clause.
     *
     * Note: Do not include a trailing period ('.') in the fact string. If a
     * period is present, it will be automatically removed.
     *
     * @param p_fact The Prolog fact to remove (must match exactly).
     * @return true if a matching fact was found and removed, false otherwise.
     *
     * @example
     * prolog.retract_fact("parent(tom, bob)")
     * prolog.retract_fact("game_state(level, 5)")
     */
    bool retract_fact(String const& p_fact);

    /**
     * @brief Retracts all facts matching a functor pattern.
     *
     * This method removes all clauses that match the given functor pattern.
     * Uses Prolog's retractall/1 predicate, which removes all matching clauses.
     *
     * Note: Do not include a trailing period ('.') in the functor pattern. If a
     * period is present, it will be automatically removed.
     *
     * @param p_functor The functor pattern to match (can contain variables).
     * @return true if any matching facts were retracted, false otherwise.
     *
     * @example
     * prolog.retract_all("likes(_, _)")      // Remove all likes/2 facts
     * prolog.retract_all("parent(tom, _)")   // Remove all facts with tom as first arg
     */
    bool retract_all(String const& p_functor);

    // =========================================================================
    // Predicate Manipulation
    // =========================================================================

    /**
     * @brief Calls a Prolog predicate with arguments.
     *
     * This method constructs a Prolog goal from a predicate name and arguments,
     * then executes it. The arguments are converted from C++ types to
     * Prolog terms automatically.
     *
     * @param p_predicate Name of the predicate to call (e.g., "member").
     * @param p_args Array of arguments to pass to the predicate.
     * @return true if the predicate call succeeded, false otherwise.
     *
     * @example
     * prolog.call_predicate("member", {3, std::vector{1, 2, 3, 4}})  // Returns true
     */
    bool call_predicate(String const& p_predicate, Array const& p_args);

    /**
     * @brief Calls a Prolog predicate and returns a result value.
     *
     * This method is similar to call_predicate(), but treats the predicate
     * as a function that returns a value. The result is expected to be the
     * last argument of the predicate.
     *
     * @param p_predicate Name of the predicate to call.
     * @param p_args Array of input arguments (result is the last argument).
     * @return The result value as a Variant, or empty Variant if the call failed.
     *
     * @example
     * auto len = prolog.call_function("length", {std::vector{1, 2, 3}})
     * // Returns: 3
     */
    Variant call_function(String const& p_predicate, Array const& p_args);

    // =========================================================================
    // Introspection
    // =========================================================================

    /**
     * @brief Checks if a predicate exists with the given arity.
     *
     * This method uses PL_predicate() to look up a predicate. If the predicate
     * doesn't exist, PL_predicate() returns 0 (NULL).
     *
     * @param p_predicate Name of the predicate to check.
     * @param p_arity Number of arguments the predicate should have.
     * @return true if the predicate exists, false otherwise.
     */
    bool predicate_exists(String const& p_predicate, int p_arity);

    /**
     * @brief Lists all currently defined predicates.
     *
     * This method uses Prolog's current_predicate/1 to query for all predicates
     * currently in the knowledge base. Returns an Array of results from
     * query_all().
     *
     * @return Array of Dictionary objects describing each predicate.
     */
    Array list_predicates();

private:

    /**
     * @brief Resolves file paths (for compatibility, returns path as-is).
     *
     * @param p_path The path to resolve.
     * @return The resolved absolute path.
     */
    static String resolve_path(String const& p_path);

    /**
     * @brief Resolves the SWI-Prolog home directory from the "home" option.
     *
     * Priority: 1) User-specified "home", 2) Empty (system default).
     * Verifies boot.prc exists. Sets error message if user explicitly
     * provided an invalid path.
     *
     * @param p_home_option Value of the "home" option (may be empty or absolute).
     * @return Resolved absolute path with boot.prc, or empty for system
     * default, and error message if validation fails.
     */
    std::pair<String, String> set_swi_home_dir(String const& p_home_option);

    /**
     * @brief Converts a Prolog term to a C++ Variant.
     *
     * Internal helper method for converting Prolog data types to C++ types.
     * Handles atoms, integers, floats, strings, lists, and compound terms.
     *
     * @param p_term The Prolog term to convert.
     * @return The converted Variant.
     */
    Variant term_to_variant(term_t p_term);

    /**
     * @brief Converts a C++ Variant to a Prolog term.
     *
     * Internal helper method for converting C++ types to Prolog terms.
     * Handles bool, int, double, String, and Array types.
     *
     * @param p_var The Variant to convert.
     * @return The created Prolog term (0 if conversion failed).
     */
    term_t variant_to_term(Variant const& p_var);

    /**
     * @brief Helper to create Prolog lists from C++ Arrays.
     *
     * @param p_arr The Array to convert.
     * @return The created Prolog list term.
     */
    term_t array_to_prolog_list(Array const& p_arr);

    /**
     * @brief Helper to construct a Prolog query from predicate name and
     * arguments.
     *
     * If p_args is empty, returns p_predicate as-is (assumed to be a full
     * goal). Otherwise, constructs a query like "predicate(arg1, arg2, ...)".
     *
     * @param p_predicate The predicate name.
     * @param p_args Array of variable names or values.
     * @return The constructed query string.
     */
    String build_query(String const& p_predicate, Array const& p_args);

    /**
     * @brief Helper to extract variable bindings from a Prolog term.
     *
     * Extracts the values of specified variables from a query result term.
     *
     * @param p_term The Prolog term containing the solution.
     * @param p_variables Array of variable names to extract.
     * @return Dictionary mapping variable names to their values.
     */
    Dictionary extract_variables(term_t p_term, Array const& p_variables);

    /**
     * @brief Helper to push error messages respecting error handling options.
     *
     * This method checks the error handling options ("on error", "on warning")
     * and either prints the error, halts, or only stores it in m_last_error.
     *
     * @param p_message The error message to handle.
     * @param p_type The error type ("error" or "warning").
     */
    void push_error(String const& p_message, String const& p_type = "error");

    /**
     * @brief Helper to handle Prolog exceptions.
     *
     * This method extracts exception information from a failed query and
     * stores it in m_last_error, then displays it as an error message.
     *
     * @param p_qid The query ID that raised the exception.
     * @param p_context Context string for error messages (e.g., "Query",
     * "Load file").
     * @return true if exception was handled, false if no exception occurred.
     */
    bool handle_prolog_exception(qid_t p_qid, String const& p_context);

    // Helper methods for Dictionary access
    template<typename T>
    T get_option(Dictionary const& dict, String const& key, T default_value) const;

    bool has_option(Dictionary const& dict, String const& key) const;

private:

    /** Whether the Prolog engine has been initialized. */
    bool m_initialized;

    /** Last error message from Prolog. */
    String m_last_error;

    /** Error handling option: "print", "halt", or "status". */
    String m_on_error;

    /** Warning handling option: "print", "halt", or "status". */
    String m_on_warning;

    /**
     * @brief Singleton instance pointer for global access.
     *
     * This static member allows access to the Prolog instance from anywhere
     * in the codebase without needing to pass references around.
     */
    static Prolog* m_singleton;
};

} // namespace robotik::prolog

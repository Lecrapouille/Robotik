#include "Robotik/Core/Prolog/Prologot.hpp"

#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#ifdef _WIN32
#    include <cstdlib>   // For _putenv_s on Windows
#    include <windows.h> // For GetModuleFileName
#else
#    include <dlfcn.h>  // For dladdr to find library path
#    include <unistd.h> // For setenv on Unix
#endif

namespace robotik::prolog {

// =============================================================================
// Static Member Initialization
// =============================================================================

Prolog* Prolog::m_singleton = nullptr;

// =============================================================================
// Constructor and Destructor
// =============================================================================

Prolog::Prolog()
{
    m_initialized = false;
    m_on_error = "print";
    m_on_warning = "print";
    m_singleton = this;
}

Prolog::~Prolog()
{
    cleanup();
    m_singleton = nullptr;
}

// =============================================================================
// Helper methods for Dictionary access
// =============================================================================

template<typename T>
T Prolog::get_option(Dictionary const& dict, String const& key, T default_value) const
{
    auto it = dict.find(key);
    if (it == dict.end())
        return default_value;

    try
    {
        return std::any_cast<T>(it->second);
    }
    catch (std::bad_any_cast const&)
    {
        return default_value;
    }
}

bool Prolog::has_option(Dictionary const& dict, String const& key) const
{
    return dict.find(key) != dict.end();
}

// =============================================================================
// Initialization and Cleanup
// =============================================================================

String Prolog::resolve_path(String const& p_path)
{
    return p_path;
}

std::pair<String, String>
Prolog::set_swi_home_dir(String const& p_home_option)
{
    if (p_home_option.empty())
        return std::make_pair("", ""); // Use system default SWI-Prolog

    String path = resolve_path(p_home_option);

    if (!std::filesystem::exists(path + "/boot.prc"))
    {
        String out_error = "boot.prc not found in: " + path;
        return std::make_pair("", out_error);
    }

    return std::make_pair(path, "");
}

bool Prolog::initialize(Dictionary const& p_options)
{
    // Idempotent: if already initialized, return success immediately
    if (m_initialized)
        return true;

    // Extract other options
    bool quiet = get_option<bool>(p_options, "quiet", true);
    bool optimized = get_option<bool>(p_options, "optimized", false);
    bool traditional = get_option<bool>(p_options, "traditional", false);
    bool threads = get_option<bool>(p_options, "threads", true);
    bool packs = get_option<bool>(p_options, "packs", true);
    String stack_limit = get_option<String>(p_options, "stack limit", "");
    String table_space = get_option<String>(p_options, "table space", "");
    String shared_table_space = get_option<String>(p_options, "shared table space", "");
    String init_file = get_option<String>(p_options, "init file", "");
    String script_file = get_option<String>(p_options, "script file", "");
    String toplevel = get_option<String>(p_options, "toplevel", "");
    m_on_error = get_option<String>(p_options, "on error", "print");
    m_on_warning = get_option<String>(p_options, "on warning", "print");

    // Extract and resolve home directory
    auto [home, error] = set_swi_home_dir(get_option<String>(p_options, "home", ""));
    if (!error.empty())
    {
        m_last_error = error;
        push_error("Invalid SWI-Prolog home directory: " + error +
                   ". I will try to use the default one.");
    }

    // Build argv for PL_initialise
    std::vector<std::string> string_storage;
    std::vector<const char*> argv_list;
    argv_list.push_back("robotik");

    if (!home.empty())
    {
        string_storage.push_back("--home=" + home);
        argv_list.push_back(string_storage.back().c_str());
    }

    // Boolean options
    if (quiet)
        argv_list.push_back("--quiet");
    if (optimized)
        argv_list.push_back("-O");
    if (traditional)
        argv_list.push_back("--traditional");
    if (!threads)
        argv_list.push_back("--no-threads");
    if (!packs)
        argv_list.push_back("--no-packs");

    // Options with values (format --option=value)
    if (!m_on_error.empty() && m_on_error != "print")
    {
        string_storage.push_back("--on-error=" + m_on_error);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!m_on_warning.empty() && m_on_warning != "print")
    {
        string_storage.push_back("--on-warning=" + m_on_warning);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!stack_limit.empty())
    {
        string_storage.push_back("--stack-limit=" + stack_limit);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!table_space.empty())
    {
        string_storage.push_back("--table-space=" + table_space);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!shared_table_space.empty())
    {
        string_storage.push_back("--shared-table-space=" + shared_table_space);
        argv_list.push_back(string_storage.back().c_str());
    }

    // Files and toplevel
    if (!init_file.empty())
    {
        argv_list.push_back("-f");
        string_storage.push_back(init_file);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!script_file.empty())
    {
        argv_list.push_back("-l");
        string_storage.push_back(script_file);
        argv_list.push_back(string_storage.back().c_str());
    }
    if (!toplevel.empty())
    {
        argv_list.push_back("-t");
        string_storage.push_back(toplevel);
        argv_list.push_back(string_storage.back().c_str());
    }

    // Goals (-g can be repeated)
    if (has_option(p_options, "goal"))
    {
        try
        {
            String goal = std::any_cast<String>(p_options.at("goal"));
            if (!goal.empty())
            {
                argv_list.push_back("-g");
                string_storage.push_back(goal);
                argv_list.push_back(string_storage.back().c_str());
            }
        }
        catch (std::bad_any_cast const&)
        {
            try
            {
                Array goals = std::any_cast<Array>(p_options.at("goal"));
                for (auto const& g : goals)
                {
                    String goal = std::any_cast<String>(g);
                    argv_list.push_back("-g");
                    string_storage.push_back(goal);
                    argv_list.push_back(string_storage.back().c_str());
                }
            }
            catch (std::bad_any_cast const&)
            {
                // Ignore invalid goal types
            }
        }
    }

    // Prolog flags (-D name=value)
    if (has_option(p_options, "prolog flags"))
    {
        try
        {
            Dictionary flags = std::any_cast<Dictionary>(p_options.at("prolog flags"));
            for (auto const& [key, value] : flags)
            {
                argv_list.push_back("-D");
                string_storage.push_back(key + "=" + std::any_cast<String>(value));
                argv_list.push_back(string_storage.back().c_str());
            }
        }
        catch (std::bad_any_cast const&)
        {
            // Ignore invalid flags
        }
    }

    // File search paths (-p alias=path)
    if (has_option(p_options, "file search paths"))
    {
        try
        {
            Dictionary paths = std::any_cast<Dictionary>(p_options.at("file search paths"));
            for (auto const& [alias, path] : paths)
            {
                argv_list.push_back("-p");
                string_storage.push_back(alias + "=" + std::any_cast<String>(path));
                argv_list.push_back(string_storage.back().c_str());
            }
        }
        catch (std::bad_any_cast const&)
        {
            // Ignore invalid paths
        }
    }

    // Custom arguments
    if (has_option(p_options, "custom args"))
    {
        try
        {
            Array custom_args = std::any_cast<Array>(p_options.at("custom args"));
            for (auto const& arg : custom_args)
            {
                string_storage.push_back(std::any_cast<String>(arg));
                argv_list.push_back(string_storage.back().c_str());
            }
        }
        catch (std::bad_any_cast const&)
        {
            // Ignore invalid args
        }
    }

    argv_list.push_back(nullptr);

    // Initialize Prolog engine
    if (!PL_initialise(static_cast<int>(argv_list.size() - 1),
                       const_cast<char**>(argv_list.data())))
    {
        if (!handle_prolog_exception(0, "PL_initialise"))
        {
            m_last_error = "PL_initialise() failed (no details available)";
        }

        return false;
    }

    // Log which SWI_HOME_DIR is being used by Prolog
    term_t home_term = PL_new_term_ref();
    predicate_t flag_pred = PL_predicate("current_prolog_flag", 2, "system");
    term_t flag_args = PL_new_term_refs(2);
    PL_put_atom_chars(flag_args + 0, "home");
    if (PL_call_predicate(NULL, PL_Q_NORMAL, flag_pred, flag_args))
    {
        char* home_path;
        if (PL_get_chars(flag_args + 1, &home_path, CVT_ATOM | CVT_STRING))
        {
            std::cout << "[Prolog] SWI-Prolog HOME: " << home_path << std::endl;
        }
    }
    (void)home_term; // Avoid unused warning

    // Bootstrap helper predicates for consult_string()
    const char* predicates[] = {
        "load_program_from_string(Code) :- "
        "open_string(Code, Stream), "
        "call_cleanup(Prolog_load_clauses(Stream), close(Stream))",

        "Prolog_load_clauses(Stream) :- "
        "read_term(Stream, Term, []), "
        "(Term == end_of_file -> true ; "
        "Prolog_process_clause(Term), Prolog_load_clauses(Stream))",

        "Prolog_process_clause((:- Goal)) :- !, call(Goal)",

        "Prolog_process_clause((?- Goal)) :- !, call(Goal)",

        "Prolog_process_clause(Clause) :- assertz(Clause)",

        nullptr
    };

    predicate_t assert_pred = PL_predicate("assertz", 1, "user");

    for (int i = 0; predicates[i] != nullptr; i++)
    {
        term_t clause = PL_new_term_ref();

        if (!PL_chars_to_term(predicates[i], clause))
        {
            m_last_error = String("Failed to parse bootstrap predicate: ") +
                           String(predicates[i]);
            PL_cleanup(0);
            return false;
        }

        qid_t qid =
            PL_open_query(NULL, PL_Q_CATCH_EXCEPTION, assert_pred, clause);
        int result = PL_next_solution(qid);

        if (result == PL_S_EXCEPTION || !result)
        {
            term_t ex = PL_exception(qid);
            if (ex)
            {
                char* msg;
                if (PL_get_chars(
                        ex, &msg, CVT_WRITE | BUF_DISCARDABLE | REP_UTF8))
                {
                    m_last_error =
                        String("Failed to assert bootstrap predicate: ") +
                        String(msg);
                }
                else
                {
                    m_last_error =
                        String("Failed to assert bootstrap predicate: ") +
                        String(predicates[i]);
                }
            }
            else
            {
                m_last_error =
                    String("Failed to assert bootstrap predicate: ") +
                    String(predicates[i]);
            }
            PL_close_query(qid);
            PL_cleanup(0);
            return false;
        }

        PL_close_query(qid);
    }

    m_initialized = true;
    return true;
}

void Prolog::cleanup()
{
    if (m_initialized)
    {
        PL_cleanup(0);
        m_initialized = false;
    }
}

bool Prolog::is_initialized() const
{
    return m_initialized;
}

// =============================================================================
// File and Code Consultation
// =============================================================================

bool Prolog::consult_file(String const& p_filename)
{
    if (!m_initialized)
        return false;

    if (p_filename.empty())
    {
        m_last_error = "Empty filename";
        return false;
    }

    String filename = resolve_path(p_filename);

    predicate_t pred = PL_predicate("consult", 1, "user");
    term_t args = PL_new_term_refs(1);

    if (!PL_put_atom_chars(args, filename.c_str()))
    {
        m_last_error = "Failed to convert filename to Prolog atom";
        return false;
    }

    qid_t qid = PL_open_query(NULL, PL_Q_CATCH_EXCEPTION, pred, args);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Consult");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);
    return result != 0;
}

bool Prolog::consult_string(String const& p_prolog_code)
{
    if (!m_initialized)
        return false;

    if (p_prolog_code.empty())
    {
        m_last_error = "Empty Prolog code";
        return false;
    }

    term_t t = PL_new_term_ref();
    if (!PL_put_string_chars(t, p_prolog_code.c_str()))
    {
        m_last_error = "Failed to convert code to Prolog string";
        return false;
    }

    predicate_t pred = PL_predicate("load_program_from_string", 1, "user");

    term_t args = PL_new_term_refs(1);
    if (!PL_put_term(args, t))
    {
        m_last_error = "Failed to prepare arguments";
        return false;
    }

    qid_t qid = PL_open_query(NULL, PL_Q_CATCH_EXCEPTION, pred, args);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Consult string");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);
    return result != 0;
}

// =============================================================================
// Query Execution
// =============================================================================

String Prolog::build_query(String const& p_predicate, Array const& p_args)
{
    String predicate = p_predicate;

    // Remove trailing period if present
    if (!predicate.empty() && predicate.back() == '.')
    {
        predicate.pop_back();
    }

    if (p_args.empty())
    {
        return predicate;
    }

    String query = predicate + "(";
    for (size_t i = 0; i < p_args.size(); i++)
    {
        if (i > 0)
            query += ", ";

        try
        {
            String arg = std::any_cast<String>(p_args[i]);
            if (!arg.empty() &&
                ((arg[0] >= 'A' && arg[0] <= 'Z') || arg[0] == '_'))
            {
                query += arg;
            }
            else
            {
                query += arg;
            }
        }
        catch (std::bad_any_cast const&)
        {
            try
            {
                int val = std::any_cast<int>(p_args[i]);
                query += std::to_string(val);
            }
            catch (std::bad_any_cast const&)
            {
                try
                {
                    double val = std::any_cast<double>(p_args[i]);
                    query += std::to_string(val);
                }
                catch (std::bad_any_cast const&)
                {
                    query += "_";
                }
            }
        }
    }
    query += ")";
    return query;
}

bool Prolog::query(String const& p_predicate, Array const& p_args)
{
    if (!m_initialized)
        return false;

    String goal = build_query(p_predicate, p_args);

    if (goal.empty())
    {
        m_last_error = "Empty query";
        return false;
    }

    term_t t = PL_new_term_ref();
    if (!PL_chars_to_term(goal.c_str(), t))
    {
        m_last_error = "Failed to parse query: " + goal;
        return false;
    }

    qid_t qid = PL_open_query(
        NULL, PL_Q_CATCH_EXCEPTION, PL_predicate("call", 1, "user"), t);

    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Query");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);

    return result != 0;
}

Dictionary Prolog::extract_variables(term_t p_term, Array const& p_variables)
{
    Dictionary result;

    atom_t name;
    size_t arity;
    if (!PL_get_name_arity(p_term, &name, &arity))
        return result;

    for (size_t i = 0; i < p_variables.size() && i < arity; i++)
    {
        try
        {
            String var_name = std::any_cast<String>(p_variables[i]);
            term_t arg = PL_new_term_ref();
            if (PL_get_arg(static_cast<int>(i + 1), p_term, arg))
            {
                Variant value = term_to_variant(arg);
                result[var_name] = value;
            }
        }
        catch (std::bad_any_cast const&)
        {
            // Skip invalid variable names
        }
    }

    return result;
}

Array Prolog::query_all(String const& p_predicate, Array const& p_args)
{
    Array results;
    if (!m_initialized)
        return results;

    String goal = build_query(p_predicate, p_args);

    if (goal.empty())
    {
        m_last_error = "Empty query";
        return results;
    }

    bool extract_vars = !p_args.empty();
    for (auto const& arg : p_args)
    {
        try
        {
            String var_name = std::any_cast<String>(arg);
            if (var_name.empty() ||
                ((var_name[0] < 'A' || var_name[0] > 'Z') && var_name[0] != '_'))
            {
                extract_vars = false;
                break;
            }
        }
        catch (std::bad_any_cast const&)
        {
            extract_vars = false;
            break;
        }
    }

    String findall_goal =
        "findall(" + goal + ", " + goal + ", PrologResults__)";

    term_t t = PL_new_term_ref();
    if (!PL_chars_to_term(findall_goal.c_str(), t))
    {
        m_last_error = "Failed to parse query: " + goal;
        return results;
    }

    qid_t qid = PL_open_query(
        NULL, PL_Q_CATCH_EXCEPTION, PL_predicate("call", 1, "user"), t);

    int solution_result = PL_next_solution(qid);

    if (solution_result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Query all");
        PL_close_query(qid);
        return results;
    }

    if (solution_result)
    {
        term_t findall_term = PL_new_term_ref();
        if (PL_get_arg(3, t, findall_term))
        {
            term_t head = PL_new_term_ref();
            term_t tail = PL_copy_term_ref(findall_term);

            while (PL_get_list(tail, head, tail))
            {
                if (extract_vars)
                {
                    Dictionary var_dict = extract_variables(head, p_args);
                    results.push_back(var_dict);
                }
                else
                {
                    results.push_back(term_to_variant(head));
                }
            }
        }
    }

    PL_close_query(qid);
    return results;
}

Variant Prolog::query_one(String const& p_predicate, Array const& p_args)
{
    if (!m_initialized)
        return Variant();

    String goal = build_query(p_predicate, p_args);

    if (goal.empty())
    {
        m_last_error = "Empty query";
        return Variant();
    }

    bool extract_vars = !p_args.empty();
    for (auto const& arg : p_args)
    {
        try
        {
            String var_name = std::any_cast<String>(arg);
            if (var_name.empty() ||
                ((var_name[0] < 'A' || var_name[0] > 'Z') && var_name[0] != '_'))
            {
                extract_vars = false;
                break;
            }
        }
        catch (std::bad_any_cast const&)
        {
            extract_vars = false;
            break;
        }
    }

    term_t t = PL_new_term_ref();
    if (!PL_chars_to_term(goal.c_str(), t))
    {
        m_last_error = "Failed to parse query: " + goal;
        return Variant();
    }

    qid_t qid = PL_open_query(
        NULL, PL_Q_CATCH_EXCEPTION, PL_predicate("call", 1, "user"), t);

    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Query one");
        PL_close_query(qid);
        return Variant();
    }

    Variant var;
    if (result)
    {
        if (extract_vars)
        {
            var = extract_variables(t, p_args);
        }
        else
        {
            var = term_to_variant(t);
        }
    }

    PL_close_query(qid);
    return var;
}

// =============================================================================
// Dynamic Assertions
// =============================================================================

bool Prolog::add_fact(String const& p_fact)
{
    if (!m_initialized)
        return false;

    if (p_fact.empty())
    {
        m_last_error = "Empty fact";
        return false;
    }

    String fact = p_fact;
    if (!fact.empty() && fact.back() == '.')
    {
        fact.pop_back();
    }

    term_t t = PL_new_term_ref();
    if (!PL_chars_to_term(fact.c_str(), t))
    {
        m_last_error = "Failed to parse fact: " + fact;
        return false;
    }

    predicate_t pred = PL_predicate("assert", 1, "user");

    qid_t qid = PL_open_query(NULL, PL_Q_CATCH_EXCEPTION, pred, t);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Assert fact");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);
    return result != 0;
}

bool Prolog::retract_fact(String const& p_fact)
{
    if (!m_initialized)
        return false;

    if (p_fact.empty())
    {
        m_last_error = "Empty fact";
        return false;
    }

    String fact = p_fact;
    if (!fact.empty() && fact.back() == '.')
    {
        fact.pop_back();
    }

    term_t t = PL_new_term_ref();
    if (!PL_chars_to_term(fact.c_str(), t))
    {
        m_last_error = "Failed to parse fact: " + fact;
        return false;
    }

    predicate_t pred = PL_predicate("retract", 1, "user");

    qid_t qid = PL_open_query(NULL, PL_Q_CATCH_EXCEPTION, pred, t);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Retract fact");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);
    return result != 0;
}

bool Prolog::retract_all(String const& p_functor)
{
    if (!m_initialized)
        return false;

    String functor = p_functor;
    if (!functor.empty() && functor.back() == '.')
    {
        functor.pop_back();
    }

    String goal = "retractall(" + functor + ")";
    return query(goal);
}

// =============================================================================
// Predicate Manipulation
// =============================================================================

bool Prolog::call_predicate(String const& p_predicate, Array const& p_args)
{
    if (!m_initialized)
        return false;

    if (p_predicate.empty())
    {
        m_last_error = "Empty predicate name";
        return false;
    }

    term_t t = PL_new_term_refs(static_cast<int>(p_args.size()));

    for (size_t i = 0; i < p_args.size(); i++)
    {
        term_t arg = variant_to_term(p_args[i]);
        if (!PL_put_term(t + static_cast<int>(i), arg))
        {
            m_last_error = "Failed to convert argument " + std::to_string(i);
            return false;
        }
    }

    functor_t f = PL_new_functor(PL_new_atom(p_predicate.c_str()),
                                 static_cast<int>(p_args.size()));

    term_t goal = PL_new_term_ref();
    if (!PL_cons_functor_v(goal, f, t))
    {
        m_last_error = "Failed to construct predicate term";
        return false;
    }

    qid_t qid = PL_open_query(
        NULL, PL_Q_CATCH_EXCEPTION, PL_predicate("call", 1, "user"), goal);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Call predicate");
        PL_close_query(qid);
        return false;
    }

    PL_close_query(qid);
    return result != 0;
}

Variant Prolog::call_function(String const& p_predicate, Array const& p_args)
{
    if (!m_initialized)
        return Variant();

    if (p_predicate.empty())
    {
        m_last_error = "Empty predicate name";
        return Variant();
    }

    term_t t = PL_new_term_refs(static_cast<int>(p_args.size() + 1));

    for (size_t i = 0; i < p_args.size(); i++)
    {
        term_t arg = variant_to_term(p_args[i]);
        if (!PL_put_term(t + static_cast<int>(i), arg))
        {
            m_last_error = "Failed to convert argument " + std::to_string(i);
            return Variant();
        }
    }

    functor_t f = PL_new_functor(PL_new_atom(p_predicate.c_str()),
                                 static_cast<int>(p_args.size() + 1));
    term_t goal = PL_new_term_ref();
    if (!PL_cons_functor_v(goal, f, t))
    {
        m_last_error = "Failed to construct predicate term";
        return Variant();
    }

    qid_t qid = PL_open_query(
        NULL, PL_Q_CATCH_EXCEPTION, PL_predicate("call", 1, "user"), goal);
    int result = PL_next_solution(qid);

    if (result == PL_S_EXCEPTION)
    {
        handle_prolog_exception(qid, "Call function");
        PL_close_query(qid);
        return Variant();
    }

    Variant var;
    if (result)
    {
        var = term_to_variant(t + static_cast<int>(p_args.size()));
    }

    PL_close_query(qid);
    return var;
}

// =============================================================================
// Introspection
// =============================================================================

bool Prolog::predicate_exists(String const& p_predicate, int p_arity)
{
    if (!m_initialized)
        return false;

    predicate_t pred =
        PL_predicate(p_predicate.c_str(), p_arity, NULL);
    return pred != 0;
}

Array Prolog::list_predicates()
{
    Array predicates;
    if (!m_initialized)
        return predicates;

    String goal = "current_predicate(Name/Arity)";
    Array results = query_all(goal);

    return results;
}

// =============================================================================
// Term Conversion
// =============================================================================

Variant Prolog::term_to_variant(term_t p_term)
{
    int type = PL_term_type(p_term);

    switch (type)
    {
        case PL_VARIABLE:
            return Variant();

        case PL_ATOM:
        {
            char* s;
            if (!PL_get_atom_chars(p_term, &s))
                return Variant();
            return String(s);
        }

        case PL_INTEGER:
        {
            int64_t i;
            if (!PL_get_int64(p_term, &i))
                return Variant();
            return i;
        }

        case PL_FLOAT:
        {
            double d;
            if (!PL_get_float(p_term, &d))
                return Variant();
            return d;
        }

        case PL_STRING:
        {
            char* s;
            size_t len;
            if (!PL_get_string_chars(p_term, &s, &len))
                return Variant();
            return String(s);
        }

        case PL_NIL:
        {
            return Array();
        }

        case PL_LIST_PAIR:
        {
            Array list_array;
            term_t head = PL_new_term_ref();
            term_t tail = PL_copy_term_ref(p_term);

            while (PL_get_list(tail, head, tail))
            {
                list_array.push_back(term_to_variant(head));
            }

            return list_array;
        }

        case PL_TERM:
        {
            term_t list_copy = PL_copy_term_ref(p_term);
            term_t head = PL_new_term_ref();
            term_t tail = PL_new_term_ref();

            if (PL_get_list(list_copy, head, tail))
            {
                Array list_array;
                list_array.push_back(term_to_variant(head));

                while (PL_get_list(tail, head, tail))
                {
                    list_array.push_back(term_to_variant(head));
                }

                return list_array;
            }

            atom_t name;
            size_t arity;
            if (PL_get_name_arity(p_term, &name, &arity))
            {
                const char* atom_name = PL_atom_chars(name);
                if (arity == 0 && strcmp(atom_name, "[]") == 0)
                {
                    return Array();
                }

                Dictionary compound;
                compound["functor"] = String(atom_name);

                Array args;
                for (size_t i = 1; i <= arity; i++)
                {
                    term_t arg = PL_new_term_ref();
                    if (!PL_get_arg(static_cast<int>(i), p_term, arg))
                    {
                        return Variant();
                    }
                    args.push_back(term_to_variant(arg));
                }
                compound["args"] = args;
                return compound;
            }
        }
    }

    return Variant();
}

term_t Prolog::variant_to_term(Variant const& p_var)
{
    term_t t = PL_new_term_ref();

    if (!p_var.has_value())
    {
        if (!PL_put_atom_chars(t, "[]"))
        {
            return (term_t)0;
        }
        return t;
    }

    // Try bool
    try
    {
        bool val = std::any_cast<bool>(p_var);
        if (!PL_put_atom_chars(t, val ? "true" : "false"))
        {
            return (term_t)0;
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try int
    try
    {
        int val = std::any_cast<int>(p_var);
        if (!PL_put_int64(t, val))
        {
            return (term_t)0;
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try int64_t
    try
    {
        int64_t val = std::any_cast<int64_t>(p_var);
        if (!PL_put_int64(t, val))
        {
            return (term_t)0;
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try double
    try
    {
        double val = std::any_cast<double>(p_var);
        if (!PL_put_float(t, val))
        {
            return (term_t)0;
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try String
    try
    {
        String val = std::any_cast<String>(p_var);
        if (!PL_put_atom_chars(t, val.c_str()))
        {
            return (term_t)0;
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try Array
    try
    {
        Array arr = std::any_cast<Array>(p_var);
        if (arr.empty())
        {
            if (!PL_put_nil(t))
            {
                return (term_t)0;
            }
        }
        else
        {
            term_t list = PL_new_term_ref();
            if (!PL_put_nil(list))
            {
                return (term_t)0;
            }

            for (auto it = arr.rbegin(); it != arr.rend(); ++it)
            {
                term_t elem = variant_to_term(*it);
                if (!elem)
                {
                    return (term_t)0;
                }
                term_t new_list = PL_new_term_ref();
                if (!PL_cons_list(new_list, elem, list))
                {
                    return (term_t)0;
                }
                list = new_list;
            }

            if (!PL_put_term(t, list))
            {
                return (term_t)0;
            }
        }
        return t;
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Try Dictionary (for compound terms)
    try
    {
        Dictionary dict = std::any_cast<Dictionary>(p_var);
        auto func_it = dict.find("functor");
        auto args_it = dict.find("args");

        if (func_it != dict.end() && args_it != dict.end())
        {
            String functor = std::any_cast<String>(func_it->second);
            Array args_arr = std::any_cast<Array>(args_it->second);
            int arity = static_cast<int>(args_arr.size());

            functor_t f = PL_new_functor(
                PL_new_atom(functor.c_str()), arity);

            term_t args = PL_new_term_refs(arity);

            for (int i = 0; i < arity; i++)
            {
                term_t arg = variant_to_term(args_arr[i]);
                if (!arg || !PL_put_term(args + i, arg))
                {
                    return (term_t)0;
                }
            }

            if (!PL_cons_functor_v(t, f, args))
            {
                return (term_t)0;
            }
            return t;
        }
    }
    catch (std::bad_any_cast const&)
    {
    }

    // Default: empty list
    if (!PL_put_atom_chars(t, "[]"))
    {
        return (term_t)0;
    }
    return t;
}

term_t Prolog::array_to_prolog_list(Array const& p_arr)
{
    term_t list = PL_new_term_ref();
    if (!PL_put_nil(list))
        return (term_t)0;

    for (auto it = p_arr.rbegin(); it != p_arr.rend(); ++it)
    {
        term_t elem = variant_to_term(*it);
        if (!elem)
            return (term_t)0;

        term_t new_list = PL_new_term_ref();
        if (!PL_cons_list(new_list, elem, list))
            return (term_t)0;
        list = new_list;
    }

    return list;
}

// =============================================================================
// Exception Handling
// =============================================================================

void Prolog::push_error(String const& p_message, String const& p_type)
{
    m_last_error = p_message;

    String option = (p_type == "warning") ? m_on_warning : m_on_error;

    if (option == "print")
    {
        std::cerr << "Prolog: " << p_message << std::endl;
    }
    else if (option == "halt")
    {
        std::cerr << "Prolog: " << p_message << std::endl;
        std::exit(1);
    }
    // "status" option: only store in m_last_error, don't print
}

bool Prolog::handle_prolog_exception(qid_t p_qid, String const& p_context)
{
    term_t exception = PL_exception(p_qid);
    if (exception)
    {
        char* exception_str;
        if (PL_get_chars(exception,
                         &exception_str,
                         CVT_WRITE | CVT_EXCEPTION | BUF_DISCARDABLE))
        {
            String error_msg =
                p_context + " error: " + String(exception_str);
            push_error(error_msg, "error");
            return true;
        }
        return false;
    }
    return false;
}

// =============================================================================
// Singleton Access
// =============================================================================

Prolog* Prolog::get_singleton()
{
    return m_singleton;
}

String Prolog::get_last_error() const
{
    return m_last_error;
}

} // namespace robotik::prolog

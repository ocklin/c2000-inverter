// SPDX-License-Identifier: GPL-3.0-or-later
//
// Guard against plain int/short/long/unsigned in the control ISR path.
// On the C28x int is 16 bits, on the host 32 bits, so an overflow there can
// never show up in a host test. Fixed-width types make the width explicit.
// Usage: IsrIntWidthTest <file>...
#include <ctype.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

static std::string StripComments(const std::string& src)
{
    std::string out;
    out.reserve(src.size());
    for (size_t i = 0; i < src.size(); ++i)
    {
        if (src.compare(i, 2, "//") == 0)
        {
            while (i < src.size() && src[i] != '\n')
                ++i;
            if (i < src.size())
                out += '\n';
        }
        else if (src.compare(i, 2, "/*") == 0)
        {
            i += 2;
            while (i < src.size() && src.compare(i, 2, "*/") != 0)
            {
                if (src[i] == '\n')
                    out += '\n'; // keep line numbers
                ++i;
            }
            ++i;
        }
        else
        {
            out += src[i];
        }
    }
    return out;
}

static bool IsIdent(char c)
{
    return isalnum((unsigned char)c) || c == '_';
}

static int CheckFile(const char* path)
{
    std::ifstream in(path);
    if (!in)
    {
        std::cerr << path << ": cannot open\n";
        return 1;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    std::istringstream lines(StripComments(buf.str()));

    static const char* const words[] = { "int", "short", "long", "unsigned" };
    int         problems = 0;
    std::string line;
    for (int lineNo = 1; std::getline(lines, line); ++lineNo)
    {
        for (const char* word : words)
        {
            const std::string w(word);
            for (size_t pos = line.find(w); pos != std::string::npos;
                 pos = line.find(w, pos + 1))
            {
                const bool startOk = pos == 0 || !IsIdent(line[pos - 1]);
                const size_t end = pos + w.size();
                const bool endOk = end >= line.size() || !IsIdent(line[end]);
                if (startOk && endOk)
                {
                    std::cerr << path << ":" << lineNo << ": plain '" << w
                              << "' in ISR path, use a fixed-width type: "
                              << line << "\n";
                    ++problems;
                }
            }
        }
    }
    return problems;
}

int main(int argc, char** argv)
{
    int problems = 0;
    for (int i = 1; i < argc; ++i)
        problems += CheckFile(argv[i]);
    if (problems == 0)
        std::cout << "No plain int types in " << argc - 1 << " ISR files\n";
    return problems == 0 ? 0 : 1;
}

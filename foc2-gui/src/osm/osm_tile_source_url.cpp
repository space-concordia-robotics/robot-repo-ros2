#include "foc2-gui/osm/osm_tile_source_url.hpp"

#include <curl/curl.h>
#include <fmt/base.h>
#include <magic_enum/magic_enum.hpp>

namespace ImOsm {
    size_t onWrite(void* data, size_t size, size_t nmemb, void* userp);
    size_t onProgress(void* clientp, double dltotal, double dlnow, double ultotal, double ulnow);
    curl_socket_t onOpenSocket(void* clientp, curlsocktype purpose, const curl_sockaddr* address);

    TileSourceUrl::TileSourceUrl(const int request_limit, const bool preload, std::string user_agent)
        : TileSourceAsync(request_limit, preload), user_agent(std::move(user_agent)) {}

    TileSourceUrl::~TileSourceUrl() {}

    bool TileSourceUrl::receiveTile(const int z, const int x, const int y, TileData& tile_data) {
        CURL* curl = curl_easy_init();
        const auto url = makeUrl(z, x, y);
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, user_agent.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 32);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 32);
        // Write
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onWrite);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void*>(&tile_data));
        // Progress
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, onProgress);
        curl_easy_setopt(curl, CURLOPT_PROGRESSDATA, &tile_data);
        // Open Socket
        curl_easy_setopt(curl, CURLOPT_OPENSOCKETFUNCTION, onOpenSocket);
        curl_easy_setopt(curl, CURLOPT_OPENSOCKETDATA, &tile_data);
        const auto result = curl_easy_perform(curl);
        const bool ok = result == CURLE_OK;
        curl_easy_cleanup(curl);

        fmt::println("got {} for tile z={},x={},y={}, url: {}", magic_enum::enum_name(result), z, x, y, url);

        return ok;
    }

    size_t onWrite(void* data, const size_t size, const size_t nmemb, void* userp) {
        auto& [_, blob] = *static_cast<TileSourceAsync::TileData*>(userp);
        const auto realsize = size * nmemb;
        auto const* const dataptr = static_cast<std::byte*>(data);
        blob.insert(blob.cend(), dataptr, dataptr + realsize);
        return realsize;
    }

    size_t onProgress(void* clientp, double /*dltotal*/, double /*dlnow*/, double /*ultotal*/, double /*ulnow*/) {
        [[maybe_unused]] auto& tileData = *static_cast<TileSourceAsync::TileData*>(clientp);
        return 0;
    }

    curl_socket_t onOpenSocket(void* clientp, curlsocktype /*purpose*/, const curl_sockaddr* address) {
        [[maybe_unused]] auto& tileData = *static_cast<TileSourceAsync::TileData*>(clientp);
        return socket(address->family, address->socktype, address->protocol);
    }
}

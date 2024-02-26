//
// Created by kiran on 2/24/24.
//


#include <sys/param.h>
#include "http.h"

esp_err_t getStatusHandler(httpd_req_t *req)
{
    /* Send a simple response */
    const char resp[] = "status: OK";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

int executeCommand(char *command);

esp_err_t postExecuteHandler(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[100];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    int result = executeCommand(content);

    printf("%d", result);

    /* Send a simple response */
    const char resp[] = "URI POST Response";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}


/* URI handler structure for POST /uri */
httpd_uri_t execute_get = {
        .uri      = "/execute",
        .method   = HTTP_POST,
        .handler  = postExecuteHandler,
        .user_ctx = NULL
};

httpd_uri_t status_get = {
        .uri      = "/status",
        .method   = HTTP_GET,
        .handler  = getStatusHandler,
        .user_ctx = NULL
};

httpd_handle_t startWebserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &status_get);
        httpd_register_uri_handler(server, &execute_get);

    }
    /* If server failed to start, handle will be NULL */
    return server;
}


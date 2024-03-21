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

int executeScript(uint8_t *script, uint16_t lenght);

esp_err_t postExecuteHandler(httpd_req_t *req)
{
    char content[100];
    size_t recv_size = MIN(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0) {  /* 0 return value indicates connection closed */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

//    int result = executeScript(content, 100);
//    printf("%d", result);

    const char resp[] = "URI POST Response";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

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
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &status_get);
        httpd_register_uri_handler(server, &execute_get);

    }
    return server;
}


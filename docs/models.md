# Models

Each model runs through its own API: Claude through the Anthropic Messages API, Gemini through
`generateContent`, and open-weights models through an OpenAI-compatible server (sglang or vLLM).

## The models

| Key | Model | Provider | Sent as `model` | `--reasoning` levels (default) | Paper settings |
|---|---|---|---|---|---|
| `opus-5.5` | Claude Opus 5.5 | `anthropic` | `claude-opus-5-5` | effort `low`, `medium`, `high`, `xhigh`, `max` (`high`) | `low`, `high` |
| `opus-5` | Claude Opus 5 | `anthropic` | `claude-opus-5` | effort `low`, `medium`, `high`, `xhigh`, `max` (`high`) | `low`, `high` |
| `opus-4.5` | Claude Opus 4.5 | `anthropic` | `claude-opus-4-5-20251101` | effort `low`, `medium`, `high` (`high`) | `low`, `high` |
| `sonnet-5.5` | Claude Sonnet 5.5 | `anthropic` | `claude-sonnet-5-5` | effort `low`, `medium`, `high`, `xhigh`, `max` (`high`) | `low`, `high` |
| `sonnet-5` | Claude Sonnet 5 | `anthropic` | `claude-sonnet-5` | effort `low`, `medium`, `high`, `xhigh`, `max` (`high`) | `low`, `high` |
| `haiku-5.5` | Claude Haiku 5.5 | `anthropic` | `claude-haiku-5-5` | effort `low`, `medium`, `high`, `xhigh`, `max` (`high`) | `low`, `high` |
| `gemini-4-argon` | Gemini 4 Argon | `gemini` | `gemini-4-argon` | thinking `off`, `low`, `medium`, `high`, `max` (`high`) | `low`, `high` |
| `qwen3.6-35b-a3b` | Qwen3.6-35B-A3B | `openai-chat` | `Qwen/Qwen3.6-35B-A3B` | thinking `off`, `on`, `max` (`off`) | `off`, `max` |
| `qwen3.8-27b` | Qwen3.8-27B | `openai-chat` | `Qwen/Qwen3.8-27B` | thinking `off`, `low`, `on`, `max` (`off`) | `low`, `max` |
| `qwen3.8-flash-next` | Qwen3.8-Flash-Next | `openai-chat` | `Qwen/Qwen3.8-Flash-Next` | thinking `off`, `low`, `on`, `max` (`off`) | `low`, `max` |
| `glm-5.3-flash` | GLM-5.3 Flash | `openai-chat` | `zai-org/GLM-5.3-Flash` | effort `low`, `high`, `max` (`high`) | `low`, `high` |
| `deepseek-v4.1-flash` | DeepSeek V4.1 Flash | `openai-chat` | `deepseek-ai/DeepSeek-V4.1-Flash-1017` | effort `low`, `high`, `max` (`high`) | `low`, `high` |
| `kimi-k3` | Kimi K3 | `openai-chat` | `kimi-k3` | effort `low`, `high`, `max` (`high`) | `low`, `high` |

The `openai-chat` models are self-hosted. The paper served them with sglang; vLLM also works.
Serve each under its "Sent as" name (`--served-model-name`) with its reasoning parser, and point
`OPENAI_BASE_URL` (or a models file) at the server.

## Configuration

### Environment variables

| Variable | Use |
|---|---|
| `ANTHROPIC_API_KEY`, `ANTHROPIC_BASE_URL` | Claude key (required) and base URL |
| `GEMINI_API_KEY`, `GEMINI_BASE_URL` | Gemini key (required) and base URL |
| `OPENAI_API_KEY`, `OPENAI_BASE_URL` | key (optional) and base URL of the OpenAI-compatible server |
| `AMP_ANTHROPIC_HEADERS`, `AMP_GEMINI_HEADERS`, `AMP_OPENAI_HEADERS` | JSON object of extra request headers |
| `AMP_MODELS_FILE` | a models file (below) |
| `SSL_CERT_FILE` | a CA bundle, if not the system trust store |

```bash
export ANTHROPIC_API_KEY=...            # opus-5.5
export GEMINI_API_KEY=...               # gemini-4-argon
export OPENAI_BASE_URL=http://gpu-box:30000/v1    # qwen3.8-flash-next on your sglang server
```

The model clients do not read `HTTPS_PROXY`; behind a proxy, set a reachable `base_url` in a
models file.

### Models file

A YAML (or JSON) file with `servers` and `models`; for a model it lists, it overrides the
environment.

```yaml
servers:
  gateway:
    wire: anthropic
    base_url: https://llm-gateway.example.com/anthropic
    api_key_env: GATEWAY_KEY        # the variable holding the key; null sends none
    api_key_header: authorization   # "authorization" sends "Bearer <key>"
    headers: {X-Project: "${PROJECT_ID}"}
models:
  opus-5.5:
    server: gateway
    served_model: claude-opus-5-5
```

- `wire`: `anthropic`, `gemini` or `openai-chat`, matching the model's provider.
- `base_url`, `headers`: `${VAR}` is read from the environment; an unset one is an error.
- `api_key_env`: defaults to the provider's variable above. Never put keys in the file.
- `served_model`: the name sent to the server (default: "Sent as" above).
- `context_tokens`: the window an `openai-chat` model trims its history to.

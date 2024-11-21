from openai import OpenAI

client = OpenAI(
    base_url = "http://localhost:11435/v1",
    api_key="ollama", # required, but unused
)

response = client.chat.completions.create(
    model="llama2",
    messages=[
        {"role": "system", "content": "You are an AI assistant."},
        {"role": "user", "content": "Who won euro champion in 2023?"},
        {"role": "assistant", "content": "The Man City won in 2023"},
        {"role": "user", "content": "Where was the finale played?"}
    ]
)

print(response.choices[0].message.content)
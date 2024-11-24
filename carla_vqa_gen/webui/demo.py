import json

def filter_json(json_data, filter_keywords):
    filter_keywords = [kw.lower() for kw in filter_keywords]

    def recursive_filter(obj):
        if isinstance(obj, dict):
            result = {}
            for key, value in obj.items():
                if any(keyword in key.lower() for keyword in filter_keywords):
                    result[key] = value
                else:
                    filtered_value = recursive_filter(value)
                    if filtered_value is not None:
                        result[key] = filtered_value
            return result if result else None
        elif isinstance(obj, list):
            result = []
            for item in obj:
                
                filtered_item = recursive_filter(item)
                if filtered_item is not None:
                    result.append(filtered_item)
            return result if result else None
        else:
            return None

    filtered_data = recursive_filter(json_data)

    return filtered_data


# 示例：加载 JSON 数据并进行过滤
if __name__ == "__main__":
    sample_json = {
        "name": "Example",
        "description": "This is a test example.",
        "details": {
            "type": "example",
            "my_example": "egg",
            "tags": ["test", {"example": "gay"}, {"example": "lesbian"} ],
            "info": {
                "content": "This contains an example."
            }
        },
        "other": "Unrelated info",
        "last_example": "believe"
    }

    filter_keywords = ['example', 'test']

    # 过滤 JSON 数据
    filtered_data = filter_json(sample_json, filter_keywords)

    # 输出过滤后的数据
    print(json.dumps(filtered_data, indent=2))

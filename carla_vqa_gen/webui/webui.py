from flask import Flask, render_template, request, jsonify
from flask_wtf import FlaskForm
from wtforms import StringField, SelectField, SubmitField
from wtforms.validators import DataRequired
from json2html import json2html
import os
import json
import gzip

app = Flask(__name__)
app.secret_key = "okitasouji"

b2d_data_path = "/media/telkwevr/22729A30729A08A5/Project_/Bench2Drive-rep"
appendix_path = "/media/telkwevr/22729A30729A08A5/Project_/Carla_Chain_QA/carla_vqa_gen/vqa_dataset/outgraph/appendix"
qa_path = "/media/telkwevr/22729A30729A08A5/Project_/Carla_Chain_QA/carla_vqa_gen/vqa_dataset/outgraph"

rel_path = ""
selected_number = None
content = {}
json_content = {}

from flask import send_from_directory

@app.route(f'/media/<path:filename>')
def media_files(filename):
    return send_from_directory('/.', filename)

class RelPathForm(FlaskForm):
    rel_path = StringField("Relative Path", validators=[DataRequired()])
    json_number = SelectField("Choose JSON Number", choices=[], coerce=str)
    submit = SubmitField("Load")


@app.route("/", methods=["GET", "POST"])
def index():
    form = RelPathForm()
    global json_content
    global content
    json_content = {
        "anno_json": "not exist",
        "appendix_json": "not exist",
        "qa_json": "not exist",
    }
    content = {
        "rgb_front": "not exist",
        "rgb_top_down": "not exist",
        "anno_json": "not exist",
        "appendix_json": "not exist",
        "qa_json": "not exist",
    }

    if request.method == "POST":
        global rel_path
        rel_path = form.rel_path.data
        qa_dir = os.path.join(qa_path, rel_path)

        # Populate the JSON number choices if directory exists
        if os.path.exists(qa_dir):
            json_files = [f for f in os.listdir(qa_dir) if f.endswith(".json")]
            json_numbers = sorted(
                [f.split('.')[0] for f in json_files if f.split('.')[0].isdigit() and len(f.split('.')[0]) == 5]
            )
            form.json_number.choices = [(num, num) for num in json_numbers]
        else:
            form.json_number.choices = []

        global selected_number
        selected_number = form.json_number.data

        # Load the selected data if a number is chosen
        if selected_number:
            rgb_front_path = os.path.join(b2d_data_path, rel_path, "camera/rgb_front", f"{selected_number}.jpg")
            rgb_top_down_path = os.path.join(b2d_data_path, rel_path, "camera/rgb_top_down", f"{selected_number}.jpg")
            anno_json_path = os.path.join(b2d_data_path, rel_path, "anno", f"{selected_number}.json.gz")
            appendix_json_path = os.path.join(appendix_path, rel_path, f"{selected_number}.json")
            qa_json_path = os.path.join(qa_path, rel_path, f"{selected_number}.json")

            content["rgb_front"] = rgb_front_path if os.path.exists(rgb_front_path) else "not exist"
            content["rgb_top_down"] = rgb_top_down_path if os.path.exists(rgb_top_down_path) else "not exist"
            json_content["anno_json"] = (
                load_gzip_json(anno_json_path) if os.path.exists(anno_json_path) else "not exist"
            )
            json_content["appendix_json"] = load_json(appendix_json_path) if os.path.exists(appendix_json_path) else "not exist"
            json_content["qa_json"] = load_json(qa_json_path) if os.path.exists(qa_json_path) else "not exist"

    # Render JSON with json2html if available
    for key in ["anno_json", "appendix_json", "qa_json"]:
        if isinstance(json_content[key], (dict, list)):
            content[key] = json2html.convert(json=json_content[key])

    return render_template("index.html", form=form, selected_number=selected_number, content=content)

def filter_json_tree(json_data, filter_keywords):
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


@app.route("/filter", methods=["POST"])
def filter_json():
    data = request.json
    filter_keywords = data.get("keywords", [])
    json_key = data.get("key")
    global json_content
    print(json_content)

    if json_key not in json_content:
        return "Invalid JSON key", 400

    # 过滤指定的 JSON
    if isinstance(json_content[json_key], (dict, list)):
        filtered_json = filter_json_tree(json_content[json_key], filter_keywords)
        html_response = json2html.convert(json=filtered_json)
    else:
        html_response = "<p>not exist</p>"

    print(json_key)
    print(html_response)
    return html_response

def load_json(json_path):
    try:
        with open(json_path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return "Error loading JSON"


def load_gzip_json(gzip_path):
    try:
        with gzip.open(gzip_path, "rt", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return "Error loading GZ JSON"


if __name__ == "__main__":
    app.run(debug=True)

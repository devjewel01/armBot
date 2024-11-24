import os
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Preformatted, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors

def get_ros_file_type(filename, path):
    """Determine the type of ROS file based on location and extension."""
    if 'launch' in path:
        return 'Launch File'
    elif 'config' in path:
        return 'Configuration File'
    elif 'srv' in path:
        return 'Service Definition'
    elif 'action' in path:
        return 'Action Definition'
    elif 'urdf' in path:
        return 'URDF File'
    elif path.endswith('/src'):
        return 'Source Code'
    elif 'include' in path:
        return 'Header File'
    else:
        return 'Other'

def is_relevant_file(filename):
    """Check if the file should be included in documentation."""
    relevant_extensions = {
        '.py', '.cpp', '.hpp', '.h', '.xml', '.launch', '.yaml', 
        '.urdf', '.xacro', '.srv', '.action', '.msg', '.ino'
    }
    return any(filename.lower().endswith(ext) for ext in relevant_extensions)

def create_section_heading(text, level=1):
    """Create section heading with different styles based on level."""
    styles = getSampleStyleSheet()
    if level == 1:
        style = ParagraphStyle(
            'Heading1',
            parent=styles['Heading1'],
            fontSize=16,
            spaceAfter=20,
        )
    else:
        style = ParagraphStyle(
            'Heading2',
            parent=styles['Heading2'],
            fontSize=14,
            spaceAfter=15,
            leftIndent=20,
        )
    return Paragraph(text, style)

def process_directory(root_path, base_path):
    """Recursively process directory and return content structure."""
    content_structure = []
    
    for dirpath, dirnames, filenames in os.walk(root_path):
        # Skip hidden directories and files
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        filenames = [f for f in filenames if not f.startswith('.') and is_relevant_file(f)]
        
        if not filenames:
            continue
            
        # Get relative path for section heading
        rel_path = os.path.relpath(dirpath, base_path)
        if rel_path != '.':
            content_structure.append({
                'type': 'section',
                'text': rel_path,
                'level': len(rel_path.split(os.sep))
            })
        
        for filename in sorted(filenames):
            file_path = os.path.join(dirpath, filename)
            try:
                with open(file_path, 'r', encoding='utf-8') as file:
                    content = file.read()
                    file_type = get_ros_file_type(filename, dirpath)
                    content_structure.append({
                        'type': 'file',
                        'name': filename,
                        'content': content,
                        'file_type': file_type
                    })
            except Exception as e:
                print(f"Error reading {file_path}: {str(e)}")
    
    return content_structure

def create_pdf(project_path, output_pdf):
    """Create PDF documentation from ROS project structure."""
    doc = SimpleDocTemplate(output_pdf, pagesize=letter)
    styles = getSampleStyleSheet()
    
    # Custom styles
    code_style = ParagraphStyle(
        'CodeStyle',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=9,
        backgroundColor=colors.lightgrey,
        leftIndent=20,
    )
    
    file_info_style = ParagraphStyle(
        'FileInfo',
        parent=styles['Normal'],
        fontSize=10,
        textColor=colors.blue,
        leftIndent=20,
    )
    
    content = []
    
    # Add title
    project_name = os.path.basename(project_path)
    content.append(Paragraph(f"{project_name} Documentation", styles['Title']))
    content.append(Spacer(1, 30))
    
    # Process the project structure
    structure = process_directory(project_path, project_path)
    
    for item in structure:
        if item['type'] == 'section':
            content.append(create_section_heading(item['text'], item['level']))
        else:
            # Add file information
            content.append(Paragraph(
                f"File: {item['name']} ({item['file_type']})",
                file_info_style
            ))
            content.append(Spacer(1, 6))
            
            # Add file content
            numbered_lines = [f"{i+1}: {line}" for i, line in enumerate(item['content'].split('\n'))]
            formatted_content = '\n'.join(numbered_lines)
            content.append(Preformatted(formatted_content, code_style))
            content.append(Spacer(1, 20))
            
            # Add page break after each file for better readability
            content.append(PageBreak())
    
    # Build PDF
    doc.build(content)

if __name__ == '__main__':
    # Example usage
    project_path = 'C:/Users/jewel.nath/Desktop/armBot'  # Replace with your armBot folder path
    output_pdf = 'armbot_code.pdf'
    create_pdf(project_path, output_pdf)
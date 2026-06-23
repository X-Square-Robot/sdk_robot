def build_stylesheet() -> str:
    return """
        QWidget {
            color: #10233a;
            font-size: 14px;
        }
        #appRoot {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #f5efe4, stop:0.55 #f3f7fb, stop:1 #e9f1fb);
        }
        #heroShell, #sidebarShell, #contentShell, #logShell {
            background: rgba(255, 255, 255, 0.84);
            border: 1px solid rgba(16, 35, 58, 0.08);
            border-radius: 22px;
        }
        #heroShell {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #fff8ea, stop:0.45 #fdfefe, stop:1 #e8f3ff);
        }
        #heroEyebrow, #contentEyebrow, #pageEyebrow, #logHint {
            color: #5b6f86;
            font-size: 11px;
            font-weight: 700;
            letter-spacing: 1px;
            text-transform: uppercase;
        }
        #heroTitle {
            font-size: 24px;
            font-weight: 700;
            color: #11243d;
        }
        #heroSubtitle, #contentSubtitle, #pageSubtitle, #fieldHint, #navSubLabel {
            color: #60748b;
        }
        #contentTitle, #pageTitle {
            font-size: 22px;
            font-weight: 700;
            color: #11243d;
        }
        #pageTitle {
            font-size: 20px;
        }
        #panelTitle {
            font-size: 18px;
            font-weight: 700;
            color: #17375a;
        }
        #modeSwitch {
            background: rgba(16, 35, 58, 0.04);
            border: 1px solid rgba(16, 35, 58, 0.08);
            border-radius: 18px;
        }
        #modeSwitch QPushButton {
            min-width: 150px;
            padding: 12px 18px;
            border-radius: 14px;
            border: 1px solid transparent;
            background: transparent;
            color: #35506e;
            font-weight: 600;
        }
        #modeSwitch QPushButton:checked {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #0e68df, stop:1 #31a2ff);
            color: white;
        }
        #serverCard, #navShell {
            background: rgba(255, 255, 255, 0.74);
            border: 1px solid rgba(16, 35, 58, 0.06);
            border-radius: 18px;
        }
        #navButtonRow {
            background: transparent;
            border: 1px solid rgba(16, 35, 58, 0.04);
            border-radius: 16px;
        }
        #navBadge, #pageBadge {
            min-width: 52px;
            padding: 8px 10px;
            border-radius: 12px;
            background: #eef5ff;
            color: #1067d9;
            font-weight: 700;
            qproperty-alignment: 'AlignCenter';
        }
        #pageBadge {
            min-width: 74px;
            font-size: 13px;
            background: #0d6ce5;
            color: white;
        }
        #navLabel {
            font-size: 15px;
            font-weight: 700;
            color: #183456;
        }
        #navButtonRow QPushButton {
            padding: 10px 14px;
            border-radius: 12px;
            border: 1px solid rgba(16, 35, 58, 0.08);
            background: rgba(255, 255, 255, 0.7);
            color: #214466;
            font-weight: 600;
        }
        #navButtonRow QPushButton:checked {
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                stop:0 #0f6be2, stop:1 #24a0ff);
            color: white;
            border: none;
        }
        #divider {
            color: rgba(16, 35, 58, 0.08);
        }
        QGroupBox {
            margin-top: 12px;
            padding-top: 12px;
            border-radius: 18px;
            border: 1px solid rgba(16, 35, 58, 0.10);
            background: rgba(255, 255, 255, 0.72);
            font-weight: 700;
            color: #17375a;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 14px;
            padding: 0 6px;
        }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
            min-height: 40px;
            padding: 0 12px;
            border-radius: 12px;
            border: 1px solid rgba(16, 35, 58, 0.14);
            background: rgba(255, 255, 255, 0.92);
            selection-background-color: #0d6ce5;
        }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
            border: 1px solid #0d6ce5;
        }
        QPushButton {
            min-height: 40px;
            padding: 0 16px;
            border-radius: 12px;
            border: 1px solid rgba(16, 35, 58, 0.10);
            background: rgba(255, 255, 255, 0.92);
            color: #17375a;
            font-weight: 600;
        }
        QPushButton:hover {
            background: #f6fbff;
        }
        QPushButton:pressed {
            background: #eaf4ff;
        }
        QPushButton:disabled {
            color: #91a0b3;
            background: #edf1f5;
        }
        QPlainTextEdit#logOutput {
            background: #11161c;
            color: #d7e0ea;
            border-radius: 16px;
            border: 1px solid rgba(255, 255, 255, 0.05);
            padding: 14px;
            font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace;
            font-size: 14px;
        }
        #statusShell {
            background: transparent;
        }
        #statusChip {
            padding: 8px 12px;
            border-radius: 999px;
            background: rgba(255, 255, 255, 0.78);
            border: 1px solid rgba(16, 35, 58, 0.08);
            color: #234667;
            font-weight: 600;
        }
        QScrollArea {
            border: none;
            background: transparent;
        }
    """

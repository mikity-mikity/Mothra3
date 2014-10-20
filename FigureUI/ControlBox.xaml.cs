﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace Mothra.UI
{
    /// <summary>
    /// FigureUI.xaml の相互作用ロジック
    /// </summary>
    public partial class ControlBox : Window
    {
        public System.Collections.Generic.List<Mothra.UI.slider> listSlider;
        public newRadioButton[] radioButtonList;
        public Func<double, double> coeff = null;
        public double force = 0.02;
        private Action radio1 = null;
        public bool objective
        {
            get
            {
                return (bool)obj.IsChecked;
            }
        }
        public void setFunctionToCompute(Action func)
        {
            this.Compute.function = func;
        }
        public void setFunctionToCompute2(Action func)
        {
            this.Compute2.function = func;
        }
        public void setFunctionForRadio1(Action func)
        {
            radio1 = func;
        }
        public ControlBox()
        {
            InitializeComponent();
            listSlider = new System.Collections.Generic.List<Mothra.UI.slider>();
            this.Hide();

        }
        public void clearSliders()
        {
            this.stackPanel1.Children.Clear();
            listSlider.Clear();
        }
        public Mothra.UI.newButton addButton(string name,Action func)
        {
            Mothra.UI.newButton b=new Mothra.UI.newButton(func);
            this.stackPanel1.Children.Add(b);
            b.Text = name;
            return b;
        }
        public Mothra.UI.slider addSlider(int min, int step, int max, int val, Action<bool> fixChanged)
        {
            Mothra.UI.slider s = addSlider(min, step, max, val, "", fixChanged);
            return s;
        }
        public Mothra.UI.slider addSlider(int min, int step, int max, int val, string text,Action<bool> fixChanged)
        {
            Mothra.UI.slider s = new Mothra.UI.slider(min, step, max, val, text);
            this.stackPanel1.Children.Add(s);
            s.getSlider.ValueChanged +=new RoutedPropertyChangedEventHandler<double>(this.slider_ValueChanged);
            s.getCheckBoxInverted.Checked += new RoutedEventHandler(this.slider_Checked);
            s.getCheckBoxInverted.Unchecked += new RoutedEventHandler(this.slider_Checked);
            s.getCheckBoxFix.Checked += new RoutedEventHandler(this.slider_Checked);
            s.getCheckBoxFix.Unchecked += new RoutedEventHandler(this.slider_Checked);
            s.fixChanged = fixChanged;
            listSlider.Add(s);
            return s;
        }

        private void slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            ((Mothra.UI.slider)(from p in listSlider where (System.Object.ReferenceEquals(p.getSlider, sender)) select p).First()).update();
        }

        private void slider_Checked(object sender, RoutedEventArgs e)
        {
            foreach (var s in listSlider)
            {
                s.update();
                s.fixChanged((bool)s.getCheckBoxFix.IsChecked);
            }
        }
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            e.Cancel = true;
            this.Hide();
        }

        public void EnableRadio(int lastComputed,Action<int> func)
        {
            radioButtonList[lastComputed].radiobutton1.IsEnabled = true;
            radioButtonList[lastComputed].function = func;
            radioButtonList[lastComputed].number = lastComputed;
        }

        private void Radio1d_Checked(object sender, RoutedEventArgs e)
        {
            coeff = (t) => { return 1; };
            if (radio1 != null) radio1();
        }

        private void Radio2d_Checked(object sender, RoutedEventArgs e)
        {
            coeff = (t) => { return t; };
            if (radio1 != null) radio1();
        }

        private void Radio3d_Checked(object sender, RoutedEventArgs e)
        {
            coeff = (t) => { return 1 / t; };
            if (radio1 != null) radio1();
        }

        private void Radio4d_Checked(object sender, RoutedEventArgs e)
        {
            coeff = (t) => { return Math.Sqrt(t); };
            if (radio1 != null) radio1();
        }

        private void Radio5d_Checked(object sender, RoutedEventArgs e)
        {
            coeff = (t) => { return Math.Sqrt(1 / t); };
            if (radio1 != null) radio1();
        }
        private void Radio0e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.000;
        }
        private void Radio1e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.005;
        }

        private void Radio2e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.01;

        }

        private void Radio3e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.02;

        }

        private void Radio4e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.05;

        }

        private void Radio5e_Checked(object sender, RoutedEventArgs e)
        {
            force = 0.1;

        }
    }
}

from django.urls import path
from . import views

urlpatterns = [
    path('',views.Home, name=""),
    path('Employee',views.Login_Employee,name="Employee"),
    path('Employee/Employee_Options',views.Employee_Options,name="Employee_Options"),
    path('Employee/Employee_Options/Inventory_data_overview',views.Data_Overview,name="Inventory_data_overview"),
    path('Employee/Manual_Control',views.Manual_Control,name="Manual_Control"),
    path('Employee/AMR_Control',views.Manual_Control,name="AMR_Control"),
]